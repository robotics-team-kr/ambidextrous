"""5단계: 집어서 옮기는 순서.

    open(열기) -> approach(접근) -> descend(하강) -> grasp(집기) -> lift(들기)
        -> check_hold(잡았는지 확인) -> transfer(옮기기) -> lower(내리기)
        -> release(놓기) -> retreat(물러나기)

중간에 한 단계라도 실패하면 거기서 멈추고, 어느 단계에서 왜 실패했는지 돌려줍니다.
"""
from . import scene
from .geometry import Pose
from .grasp import GraspError, plan_top_down_grasp, tcp_to_flange
from .moveit_client import MotionError

APPROACH_HEIGHT = 0.12    # 접근할 때 큐브 중심보다 이만큼 위에서 시작 [m]
LIFT_HEIGHT = 0.15        # 들어서 옮길 때 높이 (큐브 중심 기준) [m]
PLACE_CLEARANCE = 0.003   # 내려놓을 때 바닥에서 이만큼 띄운 채로 손을 엽니다 [m]


class PickPlace:

    def __init__(self, arm, moveit, gripper, logger):
        self._arm = arm
        self._moveit = moveit
        self._gripper = gripper
        self._log = logger
        self._stage = ''     # 지금 하고 있는 단계 이름

    def setup_planning_scene(self):
        """테이블을 MoveIt 에 장애물로 등록합니다. 팔이 테이블을 피해 움직이게 됩니다."""
        self._moveit.add_box('table', scene.TABLE_SIZE, Pose(scene.TABLE_CENTER))

    def go_home(self):
        """손을 열고 팔을 기본(ready) 자세로 돌립니다."""
        self._gripper.open()
        self._moveit.move_to_joints(self._arm.group, self._arm.arm_joints,
                                    scene.READY_JOINT_POSITIONS)

    def run(self, cube_pose, place_xy):
        """cube_pose 에 있는 큐브를 집어 place_xy 에 내려놓습니다.

        (실패한 단계 이름, 이유) 를 돌려줍니다. 끝까지 성공하면 둘 다 빈 문자열 ('', '') 입니다.
        """
        # 먼저 그리퍼가 거쳐 갈 자세들을 전부 계산해 둡니다.
        try:
            grasp = plan_top_down_grasp(cube_pose, APPROACH_HEIGHT, LIFT_HEIGHT)
        except GraspError as error:
            return 'plan_grasp', str(error)

        cube_z = cube_pose.position[2]
        orientation = grasp.grasp.orientation
        above_place = Pose((place_xy[0], place_xy[1], cube_z + LIFT_HEIGHT), orientation)
        place = Pose((place_xy[0], place_xy[1], cube_z + PLACE_CLEARANCE), orientation)

        try:
            self._begin('open')
            self._gripper.open()

            self._begin('approach')       # 큐브 바로 위로 (경로는 MoveIt 이 찾음)
            self._move_free(grasp.pregrasp)

            self._begin('descend')        # 똑바로 내려가 손가락 사이에 큐브가 오게
            self._move_linear(grasp.grasp)

            self._begin('grasp')
            self._gripper.close()
            self._require_holding()

            self._begin('lift')           # 똑바로 들어 올리기
            self._move_linear(grasp.lift)

            self._begin('check_hold')     # 들다가 놓치지 않았는지 확인
            self._require_holding()

            self._begin('transfer')       # 같은 높이로 목표 위까지
            self._transfer(above_place)

            self._begin('lower')
            self._move_linear(place)

            self._begin('release')
            self._gripper.open()

            self._begin('retreat')        # 큐브를 건드리지 않게 똑바로 위로
            self._move_linear(above_place)

        except (RuntimeError, TimeoutError) as error:
            # MotionError(이동 실패), RosTimeout(응답 없음), 놓침 등이 모두 여기로 옵니다.
            self._log.error(f'[{self._arm.side}] {self._stage} failed: {error}')
            return self._stage, str(error)
        return '', ''

    def _begin(self, stage):
        self._stage = stage
        self._log.info(f'[{self._arm.side}] {stage}')

    # 아래 두 함수는 "TCP(손가락 사이)를 여기로"라는 목표를 받아,
    # MoveIt 이 알아듣는 "손목을 여기로"로 바꿔서 보냅니다.

    def _move_free(self, tcp_pose):
        self._moveit.move_to_pose(self._arm.group, self._arm.flange_link, tcp_to_flange(tcp_pose))

    def _move_linear(self, tcp_pose):
        self._moveit.move_linear(self._arm.group, self._arm.flange_link, tcp_to_flange(tcp_pose))

    def _transfer(self, tcp_pose):
        # 큐브가 테이블에 부딪히지 않도록 같은 높이에서 직선으로 옮깁니다.
        # 직선으로 갈 수 없는 경우에만 MoveIt 이 찾은 자유 경로를 씁니다.
        try:
            self._move_linear(tcp_pose)
        except MotionError as error:
            self._log.warn(f'straight transfer not possible ({error}); planning a free motion')
            self._move_free(tcp_pose)

    def _require_holding(self):
        if not self._gripper.is_holding():
            millimeters = self._gripper.finger_position() * 1000
            raise RuntimeError('fingers closed completely: nothing is held '
                               f'(finger position {millimeters:.1f} mm)')
