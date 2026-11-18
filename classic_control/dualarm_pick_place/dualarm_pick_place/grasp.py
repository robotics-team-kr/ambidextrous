"""4단계: 큐브 자세로부터 "그리퍼를 어디로 보내야 집을 수 있는지" 계산.

방법: 위에서 아래로 집기. 그리퍼가 똑바로 아래를 보고, 두 손가락이 큐브의
마주 보는 두 면을 잡습니다. 두 손가락 사이의 가운데 점(TCP)이 큐브 중심에 오게 합니다.

여기는 순수한 계산뿐이라 ROS 없이 테스트할 수 있습니다 (test/test_grasp.py).
"""
import math

from .geometry import (
    Pose,
    box_face_yaw,
    box_tilt,
    quat_from_axis_angle,
    quat_multiply,
    rotate_vector,
    wrap_to,
)

# Panda 손의 치수 (common/dualarm_description/urdf/panda_arm.xacro 와 같아야 함).
TCP_OFFSET = 0.1034                 # 손목에서 두 손가락 사이 가운데 점(TCP)까지 거리 [m]
FLANGE_TO_HAND_YAW = -math.pi / 4   # 손은 손목에 45도 돌아간 채로 달려 있습니다
FINGER_OPEN_POSITION = 0.04         # 손가락 하나가 중심에서 4 cm -> 벌린 폭 8 cm
FINGER_CLOSED_POSITION = 0.0

Z_AXIS = (0.0, 0.0, 1.0)


class GraspError(ValueError):
    """위에서 집기로는 잡을 수 없는 자세일 때 (예: 큐브가 기울어져 있음)."""


class GraspPlan:
    """집을 때 TCP 가 거쳐 갈 세 자세. 방향은 셋 다 같고 높이만 다릅니다."""

    def __init__(self, pregrasp, grasp, lift, yaw):
        self.pregrasp = pregrasp   # 큐브 바로 위 (아직 닿지 않음)
        self.grasp = grasp         # TCP 가 큐브 중심에 온 자세
        self.lift = lift           # 잡은 뒤 똑바로 들어 올린 자세
        self.yaw = yaw             # 그리퍼를 위에서 봤을 때 돌린 각도


def top_down_orientation(yaw):
    """그리퍼가 아래를 향하고, 위에서 봤을 때 yaw 만큼 돌아간 방향.

    yaw = 0 이면 손가락이 y 방향으로 열리고 닫힙니다 ('ready' 자세와 같은 방향).
    """
    flip = quat_from_axis_angle((1.0, 0.0, 0.0), math.pi)   # 뒤집어서 아래를 보게
    spin = quat_from_axis_angle(Z_AXIS, yaw)                # 수직축으로 yaw 만큼 회전
    return quat_multiply(spin, flip)


def choose_grasp_yaw(object_yaw, preferred_yaw=0.0):
    """큐브 면과 손가락이 나란해지는 각도 중, 손목을 가장 덜 돌려도 되는 것을 고릅니다.

    큐브는 90도마다 똑같이 생겼으므로, 100도 돌아간 큐브는 10도 돌아간 것처럼 잡으면 됩니다.
    """
    return preferred_yaw + wrap_to(object_yaw - preferred_yaw, math.pi / 2)


def plan_top_down_grasp(object_pose, approach_height=0.12, lift_height=0.15):
    """테이블 위 큐브를 집기 위한 세 자세(접근 / 집기 / 들기)를 계산합니다."""
    tilt = box_tilt(object_pose.orientation)
    if tilt > math.radians(10.0):
        raise GraspError(f'object is tilted {math.degrees(tilt):.1f} deg; top-down grasp needs it flat')

    yaw = choose_grasp_yaw(box_face_yaw(object_pose.orientation))
    orientation = top_down_orientation(yaw)
    x, y, z = object_pose.position
    return GraspPlan(
        pregrasp=Pose((x, y, z + approach_height), orientation),
        grasp=Pose((x, y, z), orientation),
        lift=Pose((x, y, z + lift_height), orientation),
        yaw=yaw,
    )


def tcp_to_flange(tcp_pose):
    """"TCP 를 여기로 보내고 싶다" -> "그러려면 손목(link8)은 어디에 있어야 하나".

    MoveIt 은 손가락 끝이 아니라 손목 링크를 기준으로 계획합니다. 그래서 목표를
    보내기 전에 항상 이 함수로 바꿉니다.
      1. 위치: 손이 향하는 방향의 반대로 TCP_OFFSET 만큼 물러납니다.
      2. 방향: 손이 손목에 45도 돌아가 달려 있으므로 그만큼 되돌립니다.
    """
    hand_direction = rotate_vector(tcp_pose.orientation, Z_AXIS)
    x, y, z = tcp_pose.position
    position = (
        x - TCP_OFFSET * hand_direction[0],
        y - TCP_OFFSET * hand_direction[1],
        z - TCP_OFFSET * hand_direction[2],
    )
    undo_mount = quat_from_axis_angle(Z_AXIS, -FLANGE_TO_HAND_YAW)
    return Pose(position, quat_multiply(tcp_pose.orientation, undo_mount))
