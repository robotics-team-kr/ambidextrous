"""큐브를 한 번 집어서 목표 위치에 놓습니다.

    ros2 run pick_place_damsssss pick_place
    ros2 run pick_place_damsssss pick_place --ros-args -p arm:=left
"""
import sys

from .app import App
from .trials import check_placement


def main():
    app = App('pick_place')
    try:
        app.wait_until_ready()
        app.pick_place.go_home()

        # 1) 큐브가 어디 있는지 알아냅니다.
        cube_pose = app.objects.wait_until_still()
        x, y, z = cube_pose.position
        app.log.info(f'object at x={x:.3f} y={y:.3f} z={z:.3f}')

        # 2) 집어서 옮깁니다.
        failed_stage, reason = app.pick_place.run(cube_pose, app.arm.place_xy)
        if failed_stage:
            app.log.error(f'FAILED at {failed_stage}: {reason}')
            return 1

        # 3) 동작이 끝까지 됐더라도, 큐브가 실제로 목표에 놓였는지 시뮬레이터로 확인합니다.
        check = check_placement(app.objects.wait_until_still(), app.arm.place_xy)
        app.pick_place.go_home()
        if not check.success:
            app.log.error(f'FAILED at placement check: {check.reason}')
            return 1
        app.log.info(f'SUCCESS: placed {check.xy_error * 100:.1f} cm from the target')
        return 0
    finally:
        app.shutdown()


if __name__ == '__main__':
    sys.exit(main())
