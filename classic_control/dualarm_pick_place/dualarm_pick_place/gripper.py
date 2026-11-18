"""Open / close one Panda hand through its trajectory controller (step 3 in use)."""
from __future__ import annotations

import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

from .grasp import FINGER_CLOSED_POSITION, FINGER_OPEN_POSITION
from .ros_helpers import JointStateCache, call_action


class Gripper:

    def __init__(self, node, action_name: str, finger_joint: str, joint_states: JointStateCache,
                 move_time: float = 1.0, settle_time: float = 0.5):
        self._client = ActionClient(node, FollowJointTrajectory, action_name)
        self._action_name = action_name
        self._finger_joint = finger_joint
        self._joint_states = joint_states
        self._move_time = move_time
        self._settle_time = settle_time

    def wait_until_ready(self, timeout: float = 60.0) -> None:
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f'gripper action {self._action_name} not found')

    def open(self) -> None:
        self._command(FINGER_OPEN_POSITION)

    def close(self) -> None:
        """Close on whatever is between the fingers.

        The command is "fully closed"; with an object in the way the fingers stall
        on it and keep pressing with the joint's effort limit, which is the grip force.
        """
        self._command(FINGER_CLOSED_POSITION)

    def finger_position(self) -> float:
        """Distance of one finger from the centre [m]; the gap is twice this."""
        return self._joint_states.position(self._finger_joint)

    def is_holding(self, min_position: float = 0.005) -> bool:
        """True if something keeps the fingers from closing completely."""
        return self.finger_position() > min_position

    def _command(self, position: float) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self._finger_joint]
        seconds = int(self._move_time)
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[position],
            time_from_start=Duration(sec=seconds, nanosec=int((self._move_time - seconds) * 1e9)),
        )]
        call_action(self._client, goal, self._move_time + 10.0, 'gripper command')
        time.sleep(self._settle_time)
