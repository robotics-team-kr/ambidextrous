"""Thin wrapper over move_group's actions and services (step 2 in use).

Three kinds of motion are all the pick & place needs:
  move_to_pose    - free-space motion to a tip-link pose (OMPL plans around obstacles)
  move_to_joints  - free-space motion to joint values (e.g. the ready pose)
  move_linear     - straight-line tip motion (approach / lift / carry)
"""
from __future__ import annotations

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


class MotionError(RuntimeError):
    """Planning or execution of a motion failed."""


class MoveItClient:

    def __init__(self, node, velocity_scaling: float = 0.3, acceleration_scaling: float = 0.3,
                 linear_velocity_scaling: float = 0.1):
        self._node = node
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.linear_velocity_scaling = linear_velocity_scaling
        self._move_group = ActionClient(node, MoveGroup, '/move_action')
        self._execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self._cartesian = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._apply_scene = node.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def wait_until_ready(self, timeout: float = 60.0) -> None:
        if not self._move_group.wait_for_server(timeout_sec=timeout):
            raise MotionError('move_group is not running (action /move_action not found)')
        if not self._execute.wait_for_server(timeout_sec=timeout):
            raise MotionError('action /execute_trajectory not found')
        for client in (self._cartesian, self._apply_scene):
            if not client.wait_for_service(timeout_sec=timeout):
                raise MotionError(f'service {client.srv_name} not found')

    # ---- planning scene -------------------------------------------------------

    def add_box(self, name: str, size: tuple[float, float, float], center: Pose) -> None:
        """Tell MoveIt about a box-shaped obstacle so plans avoid it."""
        box = CollisionObject()
        box.header.frame_id = scene.WORLD_FRAME
        box.id = name
        box.pose = to_pose_msg(center)
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])]
        box.primitive_poses = [to_pose_msg(Pose((0.0, 0.0, 0.0)))]
        box.operation = CollisionObject.ADD

        planning_scene = PlanningScene(is_diff=True)
        planning_scene.world.collision_objects = [box]
        request = ApplyPlanningScene.Request(scene=planning_scene)
        response = wait_for(self._apply_scene.call_async(request), 10.0, 'apply_planning_scene')
        if not response.success:
            raise MotionError(f'MoveIt refused to add collision object {name!r}')

    # ---- free-space motions -----------------------------------------------------

    def move_to_pose(self, group: str, link: str, pose: Pose) -> None:
        position = PositionConstraint()
        position.header.frame_id = scene.WORLD_FRAME
        position.link_name = link
        position.constraint_region.primitives = [
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])]
        position.constraint_region.primitive_poses = [to_pose_msg(Pose(pose.position))]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = scene.WORLD_FRAME
        orientation.link_name = link
        orientation.orientation = to_pose_msg(pose).orientation
        orientation.absolute_x_axis_tolerance = 0.01
        orientation.absolute_y_axis_tolerance = 0.01
        orientation.absolute_z_axis_tolerance = 0.01
        orientation.weight = 1.0

        goal = Constraints(position_constraints=[position], orientation_constraints=[orientation])
        self._plan_and_execute(group, goal, f'move {link} to pose')

    def move_to_joints(self, group: str, joints: tuple[str, ...], positions: tuple[float, ...]) -> None:
        goal = Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(value),
                            tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            for name, value in zip(joints, positions)
        ])
        self._plan_and_execute(group, goal, f'move {group} to joint target')

    def _plan_and_execute(self, group: str, goal_constraints: Constraints, what: str) -> None:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.workspace_parameters.header.frame_id = scene.WORLD_FRAME
        (request.workspace_parameters.min_corner.x, request.workspace_parameters.min_corner.y,
         request.workspace_parameters.min_corner.z) = (-2.0, -2.0, -2.0)
        (request.workspace_parameters.max_corner.x, request.workspace_parameters.max_corner.y,
         request.workspace_parameters.max_corner.z) = (2.0, 2.0, 2.0)
        request.start_state.is_diff = True          # start from wherever the robot is now
        request.goal_constraints = [goal_constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        status, result = call_action(self._move_group, goal, 120.0, what)
        self._check(status, result.error_code, what)

    # ---- straight-line motion ---------------------------------------------------

    def move_linear(self, group: str, link: str, target: Pose, step: float = 0.005) -> None:
        """Move the link in a straight line from where it is now to target."""
        what = f'linear move of {link}'
        request = GetCartesianPath.Request()
        request.header.frame_id = scene.WORLD_FRAME
        request.start_state.is_diff = True
        request.group_name = group
        request.link_name = link
        request.waypoints = [to_pose_msg(target)]
        request.max_step = step
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.linear_velocity_scaling
        request.max_acceleration_scaling_factor = self.linear_velocity_scaling

        response = wait_for(self._cartesian.call_async(request), 30.0, 'compute_cartesian_path')
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionError(f'{what}: planning failed (MoveIt error {response.error_code.val})')
        if response.fraction < 0.999:
            raise MotionError(
                f'{what}: only {response.fraction:.0%} of the straight line is reachable '
                'or collision-free')

        goal = ExecuteTrajectory.Goal(trajectory=response.solution)
        status, result = call_action(self._execute, goal, 120.0, what)
        self._check(status, result.error_code, what)

    @staticmethod
    def _check(status: int, error_code: MoveItErrorCodes, what: str) -> None:
        if status != GoalStatus.STATUS_SUCCEEDED or error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionError(f'{what} failed (MoveIt error {error_code.val}, action status {status})')
