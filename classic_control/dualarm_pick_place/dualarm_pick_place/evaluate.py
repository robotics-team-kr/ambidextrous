"""6단계: 여러 번 반복해서 성공률을 잽니다.

    ros2 run dualarm_pick_place evaluate --ros-args -p case:=1 -p trials:=10
    ros2 run dualarm_pick_place evaluate --ros-args -p case:=2 -p trials:=10 -p seed:=0

Case 1: 큐브가 항상 고정 위치.
Case 2: 30 x 30 cm 정사각형 안 무작위 위치 (목표: 10번 중 8번).

성공 = 동작이 끝까지 됐고, 시뮬레이터에서 본 큐브가 목표 위치에 평평하게 놓여 있음.
결과는 <output_dir>/case<N>_<시각>.csv 로 저장됩니다.
"""
import csv
import random
import sys
import time
from datetime import datetime
from pathlib import Path

from . import scene
from .app import App
from .geometry import box_face_yaw
from .trials import check_placement, cube_start_pose, summarize

COLUMNS = ['trial', 'success', 'start_x', 'start_y', 'start_yaw', 'final_x', 'final_y', 'final_z',
           'xy_error', 'failed_stage', 'reason', 'duration_s']


def run_trial(app, start_pose):
    """한 번 시행하고, CSV 한 줄에 들어갈 내용을 dict 로 돌려줍니다."""
    row = {
        'success': False,
        'start_x': round(start_pose.position[0], 4),
        'start_y': round(start_pose.position[1], 4),
        'start_yaw': round(box_face_yaw(start_pose.orientation), 4),
    }
    started = time.monotonic()
    try:
        # 팔을 기본 자세로 돌리고, 큐브를 시작 위치로 옮긴 뒤, 집어서 옮깁니다.
        app.pick_place.go_home()
        app.sim.set_object_pose(scene.CUBE_NAME, start_pose)
        cube_pose = app.objects.wait_until_still()
        failed_stage, reason = app.pick_place.run(cube_pose, app.arm.place_xy)
        final_pose = app.objects.wait_until_still()
    except (RuntimeError, TimeoutError) as error:
        # 집기 동작 바깥(준비 과정)에서 문제가 난 경우.
        row['failed_stage'] = 'setup'
        row['reason'] = str(error)
        row['duration_s'] = round(time.monotonic() - started, 1)
        return row

    # 동작이 끝까지 됐더라도 큐브의 최종 위치로 한 번 더 판정합니다.
    check = check_placement(final_pose, app.arm.place_xy)
    if not failed_stage and not check.success:
        failed_stage, reason = 'placement_check', check.reason

    row['success'] = (failed_stage == '')
    row['final_x'] = round(final_pose.position[0], 4)
    row['final_y'] = round(final_pose.position[1], 4)
    row['final_z'] = round(final_pose.position[2], 4)
    row['xy_error'] = round(check.xy_error, 4)
    row['failed_stage'] = failed_stage
    row['reason'] = reason
    row['duration_s'] = round(time.monotonic() - started, 1)
    return row


def main():
    app = App('pick_place_eval')

    # 실행할 때 바꿀 수 있는 값들:  --ros-args -p 이름:=값
    case = app.node.declare_parameter('case', 1).value
    trials = app.node.declare_parameter('trials', 10).value
    seed = app.node.declare_parameter('seed', 0).value                    # 같은 seed -> 같은 무작위 위치
    random_yaw = app.node.declare_parameter('random_yaw', False).value    # 큐브 방향도 무작위로
    target_rate = app.node.declare_parameter('target_rate', 0.8).value
    output_dir = Path(app.node.declare_parameter('output_dir', 'results').value)

    try:
        if case not in (1, 2):
            app.log.error(f'case must be 1 or 2, got {case}')
            return 2
        app.wait_until_ready()

        rng = random.Random(seed)
        rows = []
        successes = 0
        for index in range(1, trials + 1):
            app.log.info(f'===== Case {case}, trial {index}/{trials} =====')
            row = run_trial(app, cube_start_pose(case, app.arm, rng, random_yaw))
            row['trial'] = index
            rows.append(row)
            if row['success']:
                successes += 1
                app.log.info(f'trial {index}: success')
            else:
                app.log.info(f"trial {index}: FAILED at {row['failed_stage']}: {row['reason']}")

        # 결과를 CSV 로 저장합니다 (빈 칸은 '' 로 채워집니다).
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f'case{case}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        with path.open('w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=COLUMNS, restval='')
            writer.writeheader()
            writer.writerows(rows)

        app.log.info(f'Case {case}: {summarize(successes, trials, target_rate)}')
        app.log.info(f'details: {path.resolve()}')
        passed = trials > 0 and successes / trials >= target_rate
        return 0 if passed else 1
    finally:
        app.shutdown()


if __name__ == '__main__':
    sys.exit(main())
