"""Step 6: run repeated trials and report the success rate.

    ros2 run dualarm_pick_place evaluate --ros-args -p case:=1 -p trials:=10
    ros2 run dualarm_pick_place evaluate --ros-args -p case:=2 -p trials:=10 -p seed:=0

Case 1: cube always at the fixed position.
Case 2: cube uniformly random inside the 30 x 30 cm square (target: 8 of 10).

A trial succeeds only if the whole sequence ran AND the simulator shows the cube
resting flat on the table within tolerance of the place target.
Results go to <output_dir>/case<N>_<timestamp>.csv.
"""
from __future__ import annotations

import csv
import random
import sys
import time
from datetime import datetime
from pathlib import Path

from . import scene
from .app import App
from .geometry import box_face_yaw
from .object_pose import get_object_pose
from .trials import check_placement, cube_start_pose, summarize

COLUMNS = ['trial', 'success', 'start_x', 'start_y', 'start_yaw', 'final_x', 'final_y', 'final_z',
           'xy_error', 'failed_stage', 'reason', 'duration_s']


def run_trial(app: App, case: int, rng: random.Random, random_yaw: bool, index: int) -> dict:
    row = dict.fromkeys(COLUMNS, '')
    row['trial'] = index
    row['success'] = False

    # Sample first so the sequence of start poses depends only on the seed,
    # not on whether earlier trials succeeded.
    start = cube_start_pose(case, app.arm, rng, random_yaw)
    row['start_x'], row['start_y'] = (round(v, 4) for v in start.position[:2])
    row['start_yaw'] = round(box_face_yaw(start.orientation), 4)

    started = time.monotonic()
    try:
        app.pick_place.go_home()
        app.sim.set_object_pose(scene.CUBE_NAME, start)
        object_pose = get_object_pose(app.objects)
        result = app.pick_place.run(object_pose, app.arm.place_xy)
        final = app.objects.wait_until_still()
    except (RuntimeError, TimeoutError) as error:   # MotionError, RosTimeout, sim errors
        row['failed_stage'] = 'setup'
        row['reason'] = str(error)
        row['duration_s'] = round(time.monotonic() - started, 1)
        return row

    check = check_placement(final, app.arm.place_xy)
    row['final_x'], row['final_y'], row['final_z'] = (round(v, 4) for v in final.position)
    row['xy_error'] = round(check.xy_error, 4)
    row['duration_s'] = round(time.monotonic() - started, 1)
    row['success'] = result.completed and check.success
    if not result.completed:
        row['failed_stage'], row['reason'] = result.failed_stage, result.message
    elif not check.success:
        row['failed_stage'], row['reason'] = 'placement_check', check.reason
    return row


def main() -> int:
    app = App('pick_place_eval')
    node = app.node
    case = node.declare_parameter('case', 1).value
    trials = node.declare_parameter('trials', 10).value
    seed = node.declare_parameter('seed', 0).value
    random_yaw = node.declare_parameter('random_yaw', False).value
    target_rate = node.declare_parameter('target_rate', 0.8).value
    output_dir = Path(node.declare_parameter('output_dir', 'results').value)

    try:
        if case not in (1, 2):
            app.log.error(f'case must be 1 or 2, got {case}')
            return 2
        app.wait_until_ready()
        rng = random.Random(seed)
        rows = []
        for index in range(1, trials + 1):
            app.log.info(f'===== Case {case}, trial {index}/{trials} =====')
            row = run_trial(app, case, rng, random_yaw, index)
            rows.append(row)
            outcome = 'success' if row['success'] else f"FAILED at {row['failed_stage']}: {row['reason']}"
            app.log.info(f'trial {index}: {outcome}')

        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f'case{case}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        with path.open('w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(rows)

        successes = sum(1 for row in rows if row['success'])
        app.log.info(f'Case {case}: {summarize(successes, trials, target_rate)}')
        app.log.info(f'details: {path.resolve()}')
        return 0 if trials and successes / trials >= target_rate else 1
    finally:
        app.shutdown()


if __name__ == '__main__':
    sys.exit(main())
