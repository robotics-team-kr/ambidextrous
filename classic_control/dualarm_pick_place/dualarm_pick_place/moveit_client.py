"""2단계: MoveIt(move_group)에게 "팔을 이렇게 움직여 달라"고 요청하는 부분.

경로 계산은 MoveIt 이 합니다. 여기서는 요청서를 채워서 보내고 결과를 확인할 뿐입니다.
움직임은 세 가지입니다.
  move_to_pose    손목을 원하는 자세로. 경로는 MoveIt 이 장애물을 피해 알아서 찾습니다.
  move_to_joints  관절 각도를 직접 지정 (ready 자세로 돌아갈 때).
  move_linear     손목을 지금 위치에서 목표까지 직선으로 (내려가기 / 들기 / 옮기기).
"""
from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive

from . import scene
from .geometry import Pose
from .ros_helpers import call_action, to_pose_msg, wait_for

# 속도 비율 (1.0 = 로봇 최고 속도). 느릴수록 시뮬레이션에서 안정적입니다.
FREE_SPEED = 0.3      # 자유 이동
LINEAR_SPEED = 0.1    # 직선 이동 (큐브 근처라서 더 천천히)


class MotionError(RuntimeError):
    """경로를 못 찾았거나, 움직이다가 실패했을 때."""


class MoveItClient:

    def __init__(self, node):
        # MoveIt 이 제공하는 창구 4개에 연결합니다.
        self._move_group = ActionClient(node, MoveGroup, '/move_action')                # 계획 + 실행
        self._execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')    # 실행만
        self._cartesian = node.create_client(GetCartesianPath, '/compute_cartesian_path')   # 직선 경로 계산
        self._apply_scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')  # 장애물 등록

    def wait_until_ready(self, timeout=60.0):
        if not self._move_group.wait_for_server(timeout_sec=timeout):
            raise MotionError('move_group is not running (action /move_action not found)')
        if not self._execute.wait_for_server(timeout_sec=timeout):
            raise MotionError('action /execute_trajectory not found')
        if not self._cartesian.wait_for_service(timeout_sec=timeout):
            raise MotionError('service /compute_cartesian_path not found')
        if not self._apply_scene.wait_for_service(timeout_sec=timeout):
            raise MotionError('service /apply_planning_scene not found')

    def add_box(self, name, size, center):
        """상자 모양 장애물을 MoveIt 에 알려줍니다. 이후 경로는 이 상자를 피해 갑니다."""
        box = CollisionObject()
        box.header.frame_id = scene.WORLD_FRAME
        box.id = name
        box.pose = to_pose_msg(center)
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])]
        box.primitive_poses = [to_pose_msg(Pose((0.0, 0.0, 0.0)))]
        box.operation = CollisionObject.ADD

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene(is_diff=True)   # is_diff: 기존 장면에 "추가"한다는 뜻
        request.scene.world.collision_objects = [box]
        response = wait_for(self._apply_scene.call_async(request), 10.0, 'apply_planning_scene')
        if not response.success:
            raise MotionError(f'MoveIt refused to add collision object {name!r}')

    def move_to_pose(self, group, link, pose):
        """link 가 pose 에 오도록 팔을 움직입니다 (경로는 MoveIt 이 찾음)."""
        # 목표 위치: pose 를 중심으로 한 반지름 1 mm 공 안.
        position = PositionConstraint()
        position.header.frame_id = scene.WORLD_FRAME
        position.link_name = link
        position.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])]
        position.constraint_region.primitive_poses = [to_pose_msg(Pose(pose.position))]
        position.weight = 1.0

        # 목표 방향: 각 축으로 0.01 rad 이내.
        orientation = OrientationConstraint()
        orientation.header.frame_id = scene.WORLD_FRAME
        orientation.link_name = link
        orientation.orientation = to_pose_msg(pose).orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0

        target = Constraints(position_constraints=[position], orientation_constraints=[orientation])
        self._plan_and_execute(group, target, f'move {link} to pose')

    def move_to_joints(self, group, joints, positions):
        """각 관절을 지정한 각도로 움직입니다."""
        target = Constraints()
        for name, value in zip(joints, positions):
            target.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        self._plan_and_execute(group, target, f'move {group} to joint target')

    def _plan_and_execute(self, group, target, what):
        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.goal_constraints = [target]
        goal.request.num_planning_attempts = 5      # 계획을 5번 시도해서 가장 좋은 것 사용
        goal.request.allowed_planning_time = 5.0    # 계획에 쓸 수 있는 시간 [s]
        goal.request.max_velocity_scaling_factor = FREE_SPEED
        goal.request.max_acceleration_scaling_factor = FREE_SPEED
        goal.request.start_state.is_diff = True     # 로봇의 "지금 자세"에서 출발
        goal.planning_options.plan_only = False     # 계획만 하지 말고 실제로 움직이기까지
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        status, result = call_action(self._move_group, goal, 120.0, what)
        self._check(status, result.error_code, what)

    def move_linear(self, group, link, target):
        """link 를 지금 위치에서 target 까지 직선으로 움직입니다."""
        what = f'linear move of {link}'

        # 1) 직선 경로를 계산해 달라고 요청합니다.
        request = GetCartesianPath.Request()
        request.header.frame_id = scene.WORLD_FRAME
        request.start_state.is_diff = True
        request.group_name = group
        request.link_name = link
        request.waypoints = [to_pose_msg(target)]
        request.max_step = 0.005                # 직선을 5 mm 간격으로 잘라서 계산
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = LINEAR_SPEED
        request.max_acceleration_scaling_factor = LINEAR_SPEED
        response = wait_for(self._cartesian.call_async(request), 30.0, 'compute_cartesian_path')

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionError(f'{what}: planning failed (MoveIt error {response.error_code.val})')
        # fraction: 직선 중 실제로 갈 수 있는 비율. 끝까지 못 가면 실패로 봅니다.
        if response.fraction < 0.999:
            raise MotionError(
                f'{what}: only {response.fraction:.0%} of the straight line is reachable '
                'or collision-free')

        # 2) 계산된 경로대로 움직입니다.
        goal = ExecuteTrajectory.Goal(trajectory=response.solution)
        status, result = call_action(self._execute, goal, 120.0, what)
        self._check(status, result.error_code, what)

    def _check(self, status, error_code, what):
        if status != GoalStatus.STATUS_SUCCEEDED or error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionError(f'{what} failed (MoveIt error {error_code.val}, action status {status})')
