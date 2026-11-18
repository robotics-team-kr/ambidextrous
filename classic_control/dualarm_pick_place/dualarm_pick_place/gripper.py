"""3단계: 한쪽 손(그리퍼)을 열고 닫기.

손가락은 "이 위치로 가라"는 명령으로 움직입니다 (0.04 = 활짝 벌림, 0.0 = 완전히 닫음).
닫을 때는 항상 0.0 을 명령합니다. 사이에 큐브가 있으면 손가락이 큐브에 막혀
더 못 가고, 계속 닫으려고 누르는 힘이 곧 쥐는 힘이 됩니다.
그래서 "손가락이 0 까지 못 갔다 = 뭔가를 잡고 있다"로 판단할 수 있습니다.
"""
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .grasp import FINGER_CLOSED_POSITION, FINGER_OPEN_POSITION
from .ros_helpers import RosTimeout, call_action

MOVE_TIME = 1          # 열거나 닫는 데 주는 시간 [s]
SETTLE_TIME = 0.5      # 움직인 뒤 안정될 때까지 더 기다리는 시간 [s]
HOLDING_MIN = 0.005    # 손가락이 이보다 벌어져 있으면 뭔가 잡고 있는 것 [m]


class Gripper:

    def __init__(self, node, action_name, finger_joint):
        self._action_name = action_name
        self._finger_joint = finger_joint
        self._finger_position = None   # /joint_states 에서 읽은 손가락 위치
        self._client = ActionClient(node, FollowJointTrajectory, action_name)
        self._sub = node.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        # 모든 관절 값이 한 메시지에 들어오므로 우리 손가락 것만 골라 저장합니다.
        if self._finger_joint in msg.name:
            index = list(msg.name).index(self._finger_joint)
            self._finger_position = msg.position[index]

    def wait_until_ready(self, timeout=60.0):
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f'gripper action {self._action_name} not found')

    def open(self):
        self._move_finger(FINGER_OPEN_POSITION)

    def close(self):
        self._move_finger(FINGER_CLOSED_POSITION)

    def finger_position(self):
        """손가락 하나가 중심에서 떨어진 거리 [m]. 벌린 폭은 이 값의 두 배입니다."""
        deadline = time.monotonic() + 5.0
        while self._finger_position is None:
            if time.monotonic() > deadline:
                raise RosTimeout(f'no /joint_states value for {self._finger_joint}')
            time.sleep(0.02)
        return self._finger_position

    def is_holding(self):
        """손가락이 끝까지 닫히지 않았으면 True (사이에 뭔가 있다는 뜻)."""
        return self.finger_position() > HOLDING_MIN

    def _move_finger(self, position):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self._finger_joint]
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[position], time_from_start=Duration(sec=MOVE_TIME))]
        call_action(self._client, goal, MOVE_TIME + 10.0, 'gripper command')
        time.sleep(SETTLE_TIME)
