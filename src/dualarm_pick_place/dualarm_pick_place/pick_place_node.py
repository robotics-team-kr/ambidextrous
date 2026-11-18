"""Pick the cube once from wherever it is and place it on the target.

    ros2 run dualarm_pick_place pick_place
    ros2 run dualarm_pick_place pick_place --ros-args -p arm:=left
"""
from __future__ import annotations

import sys

from .app import App
from .object_pose import get_object_pose
from .trials import check_placement


def main() -> int:
    app = App('pick_place')
    try:
        app.wait_until_ready()
        app.pick_place.go_home()

        object_pose = get_object_pose(app.objects)
        x, y, z = object_pose.position
        app.log.info(f'object at x={x:.3f} y={y:.3f} z={z:.3f}')

        result = app.pick_place.run(object_pose, app.arm.place_xy)
        check = check_placement(app.objects.wait_until_still(), app.arm.place_xy)
        success = result.completed and check.success
        if success:
            app.log.info(f'SUCCESS: placed {check.xy_error * 100:.1f} cm from the target '
                         f'in {result.duration:.1f} s')
        else:
            why = result.message if not result.completed else check.reason
            app.log.error(f'FAILED at {result.failed_stage or "placement check"}: {why}')
        if result.completed:
            app.pick_place.go_home()
        return 0 if success else 1
    finally:
        app.shutdown()


if __name__ == '__main__':
    sys.exit(main())
