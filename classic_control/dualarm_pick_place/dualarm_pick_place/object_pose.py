"""물체가 어디 있는지 알아내는 부분.  <- 카메라로 바꿀 때 이 파일만 바꾸면 됩니다.

지금은 Gazebo 가 큐브의 실제 위치를 알려줍니다 (/model/cube/pose 토픽).
카메라를 쓰게 되면, 아래 클래스와 똑같이 latest / wait_until_still 두 함수를 가진
클래스를 새로 만들어 app.py 에서 바꿔 끼우면 됩니다. 다른 코드는 그대로입니다.
"""
import math
import time

from geometry_msgs.msg import PoseStamped

from . import scene
from .ros_helpers import RosTimeout, from_pose_msg


class SimObjectPoseSource:
    """시뮬레이터가 알려주는 큐브의 실제 자세."""

    def __init__(self, node, object_name=scene.CUBE_NAME):
        self._topic = f'/model/{object_name}/pose'
        self._pose = None    # 가장 최근에 받은 자세
        self._count = 0      # 지금까지 받은 메시지 수
        self._sub = node.create_subscription(PoseStamped, self._topic, self._on_pose, 10)

    def _on_pose(self, msg):
        # 메시지가 올 때마다 배경 스레드가 이 함수를 부릅니다.
        self._pose = from_pose_msg(msg.pose)
        self._count += 1

    def latest(self, timeout=5.0):
        """지금부터 새로 도착하는 자세 하나를 기다려서 돌려줍니다.

        이미 받아 둔 값을 쓰지 않는 이유: 큐브를 방금 다른 곳으로 옮겼다면
        받아 둔 값은 옮기기 전 위치일 수 있기 때문입니다.
        """
        count_before = self._count
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._count > count_before:
                return self._pose
            time.sleep(0.01)
        raise RosTimeout(f'no object pose on {self._topic} (is the simulator running?)')

    def wait_until_still(self, timeout=10.0):
        """물체가 멈출 때까지 기다린 뒤 그 자세를 돌려줍니다."""
        deadline = time.monotonic() + timeout
        previous = self.latest()
        while time.monotonic() < deadline:
            time.sleep(0.5)
            current = self.latest()
            # 0.5초 동안 1 mm 도 안 움직였으면 멈춘 것으로 봅니다.
            if math.dist(previous.position, current.position) < 0.001:
                return current
            previous = current
        raise RosTimeout(f'object still moving after {timeout:.0f} s')
