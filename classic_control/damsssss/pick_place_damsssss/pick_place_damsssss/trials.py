"""6단계 계산 부분: 시행마다 큐브를 어디에 둘지, 결과를 어떻게 판정할지.

ROS 없이 동작하므로 실험 규칙을 시뮬레이터 없이 테스트할 수 있습니다 (test/test_trials.py).
"""
import math

from . import scene
from .geometry import Pose, box_tilt, quat_from_axis_angle


def cube_start_pose(case, arm, rng, random_yaw=False):
    """한 번의 시행에서 큐브를 처음에 둘 자세.

    Case 1: 항상 고정 위치.  Case 2: 30 x 30 cm 정사각형 안에서 무작위.
    rng 는 random.Random(seed) 로, 같은 seed 면 같은 위치들이 나옵니다.
    """
    if case == 1:
        x, y = arm.fixed_cube_xy
        yaw = 0.0
    elif case == 2:
        # 큐브 중심이 아니라 큐브 전체가 영역 안에 들어오도록 반 칸씩 안쪽에서 뽑습니다.
        margin = scene.CUBE_SIZE / 2.0
        x_min, x_max, y_min, y_max = arm.region_bounds
        x = rng.uniform(x_min + margin, x_max - margin)
        y = rng.uniform(y_min + margin, y_max - margin)
        yaw = rng.uniform(-math.pi, math.pi) if random_yaw else 0.0
    else:
        raise ValueError(f'case must be 1 or 2, got {case}')
    return Pose((x, y, scene.CUBE_REST_Z), quat_from_axis_angle((0.0, 0.0, 1.0), yaw))


class PlaceCheck:
    """놓인 결과 판정."""

    def __init__(self, success, xy_error, reason):
        self.success = success     # 성공 여부
        self.xy_error = xy_error   # 목표에서 수평으로 벗어난 거리 [m]
        self.reason = reason       # 실패 이유 (성공이면 빈 문자열)


def check_placement(final_pose, place_xy):
    """큐브가 목표 위치의 테이블 위에 평평하게 놓였는지 판정합니다."""
    x, y, z = final_pose.position
    xy_error = math.hypot(x - place_xy[0], y - place_xy[1])
    z_error = z - scene.CUBE_REST_Z

    if xy_error > scene.PLACE_TOLERANCE_XY:
        reason = (f'{xy_error * 100:.1f} cm from target '
                  f'(limit {scene.PLACE_TOLERANCE_XY * 100:.1f} cm)')
    elif abs(z_error) > scene.PLACE_TOLERANCE_Z:
        reason = f'not resting on the table (z off by {z_error * 100:.1f} cm)'
    elif box_tilt(final_pose.orientation) > math.radians(5.0):
        reason = 'not lying flat'
    else:
        reason = ''
    return PlaceCheck(success=(reason == ''), xy_error=xy_error, reason=reason)


def meets_target(successes, trials, target_rate=0.8):
    """성공률이 목표 이상이면 True. 한 번도 시행하지 않았으면 False."""
    return trials > 0 and successes / trials >= target_rate


def summarize(successes, trials, target_rate=0.8):
    """'9/10 succeeded (90%); target 80% -> PASS' 같은 요약 한 줄."""
    rate = successes / trials if trials else 0.0
    verdict = 'PASS' if meets_target(successes, trials, target_rate) else 'FAIL'
    return f'{successes}/{trials} succeeded ({rate:.0%}); target {target_rate:.0%} -> {verdict}'
