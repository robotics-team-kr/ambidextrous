"""과제 설정: 물건들이 어디 있고, 무엇을 성공으로 볼지.

여기 숫자는 Gazebo 월드 파일(common/dualarm_description/worlds/pick_place.sdf)과
같아야 합니다. test/test_scene.py 가 둘이 같은지 검사합니다.
좌표 기준은 로봇의 "world" 링크이고, 단위는 미터입니다.
"""

WORLD_FRAME = 'world'
GZ_WORLD_NAME = 'pick_place'

# 테이블: 중심이 TABLE_CENTER 인 상자. 윗면 높이는 TABLE_TOP_Z.
TABLE_SIZE = (0.6, 1.0, 0.2)
TABLE_CENTER = (0.5, 0.0, 0.1)
TABLE_TOP_Z = TABLE_CENTER[2] + TABLE_SIZE[2] / 2.0

# 집을 물체: 5 cm 큐브.
CUBE_NAME = 'cube'
CUBE_SIZE = 0.05
CUBE_REST_Z = TABLE_TOP_Z + CUBE_SIZE / 2.0   # 테이블에 놓였을 때 큐브 중심 높이

# Case 2: Case 1 위치를 중심으로 한 30 x 30 cm 정사각형 안에서 무작위.
RANDOM_REGION_SIZE = (0.30, 0.30)

# 성공 기준: 큐브가 목표에서 이 거리 안에 평평하게 놓여 있어야 합니다.
PLACE_TOLERANCE_XY = 0.03
PLACE_TOLERANCE_Z = 0.01

# 팔의 기본 자세("ready") 관절 각도 7개 [rad].
READY_JOINT_POSITIONS = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)


class ArmScene:
    """어느 팔로 집느냐에 따라 달라지는 이름과 위치 모음."""

    def __init__(self, side):
        if side not in ('left', 'right'):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        # 오른팔은 y = -0.3 쪽, 왼팔은 y = +0.3 쪽에 있습니다.
        y = -0.3 if side == 'right' else 0.3

        self.side = side
        self.group = f'{side}_panda_arm'              # MoveIt 에 등록된 팔 이름
        self.flange_link = f'{side}_panda_link8'      # 팔 끝(손목) 링크. MoveIt 은 이 링크를 기준으로 계획합니다.
        self.arm_joints = tuple(f'{side}_panda_joint{i}' for i in range(1, 8))
        self.finger_joint = f'{side}_panda_finger_joint1'
        self.hand_action = f'/{side}_hand_controller/follow_joint_trajectory'
        self.fixed_cube_xy = (0.5, y)                 # Case 1 에서 큐브가 놓이는 위치
        self.place_xy = (0.5, 0.0)                    # 큐브를 내려놓을 목표 위치

        # Case 2 에서 큐브를 뿌리는 영역 (x 최소, x 최대, y 최소, y 최대).
        half_x = RANDOM_REGION_SIZE[0] / 2.0
        half_y = RANDOM_REGION_SIZE[1] / 2.0
        self.region_bounds = (0.5 - half_x, 0.5 + half_x, y - half_y, y + half_y)
