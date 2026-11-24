"""자세(위치 + 방향) 계산에 쓰는 작은 수학 함수들. ROS 없이 동작합니다.

방향은 쿼터니언 (x, y, z, w) 4개 숫자로 나타냅니다. ROS 메시지와 같은 순서입니다.
쿼터니언이 낯설면 "회전을 나타내는 숫자 4개" 정도로만 알아도 나머지 코드를 읽을 수 있습니다.
"""
import math

NO_ROTATION = (0.0, 0.0, 0.0, 1.0)   # 회전하지 않은 상태


class Pose:
    """위치 (x, y, z) 와 방향 (쿼터니언)."""

    def __init__(self, position, orientation=NO_ROTATION):
        self.position = position
        self.orientation = orientation


def quat_from_axis_angle(axis, angle):
    """axis 축을 중심으로 angle[rad] 만큼 도는 회전."""
    length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    s = math.sin(angle / 2.0) / length
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0))


def quat_multiply(a, b):
    """회전 두 개를 합칩니다: b를 먼저 하고 그 다음 a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q, v):
    """벡터 v를 회전 q만큼 돌린 결과."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def box_axes(q):
    """방향이 q인 상자의 세 축(x, y, z)이 세상 기준으로 어디를 향하는지."""
    return [rotate_vector(q, axis) for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]


def box_tilt(q):
    """상자가 얼마나 기울어져 있는지 [rad]. 어느 면으로든 평평하게 놓여 있으면 0."""
    # 세 축 중 가장 수직에 가까운 축이 수직에서 얼마나 벗어났는지를 봅니다.
    most_vertical = max(abs(axis[2]) for axis in box_axes(q))
    return math.acos(min(1.0, most_vertical))


def box_face_yaw(q):
    """상자 옆면이 향하는 방향 [rad] (위에서 내려다본 각도).

    상자는 90도마다 똑같이 생겼으므로 이 값은 90도 단위로만 의미가 있습니다.
    """
    # 세 축 중 가장 수평에 가까운 축을 골라 그 방향을 잽니다.
    most_horizontal = box_axes(q)[0]
    for axis in box_axes(q):
        if abs(axis[2]) < abs(most_horizontal[2]):
            most_horizontal = axis
    return math.atan2(most_horizontal[1], most_horizontal[0])


def wrap_to(angle, period):
    """각도를 -period/2 ~ +period/2 범위로 접어 넣습니다."""
    return (angle + period / 2.0) % period - period / 2.0
