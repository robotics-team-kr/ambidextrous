"""Small rclpy conveniences shared by the nodes.

The nodes are written as straight-line scripts: one background thread spins the
node, and the main thread sends requests and waits for them here.
"""
from __future__ import annotations

import threading
import time

import rclpy
from geometry_msgs.msg import Pose as PoseMsg
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

from .geometry import Pose


class RosTimeout(TimeoutError):
    """A ROS server did not answer in time."""


def wait_for(future, timeout: float, what: str):
    """Block until an rclpy future completes (it is resolved by the spin thread)."""
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            raise RosTimeout(f'timed out after {timeout:.0f} s waiting for {what}')
        time.sleep(0.005)
    return future.result()


def call_action(client, goal, timeout: float, what: str):
    """Send an action goal and wait for its result. Returns (status, result)."""
    handle = wait_for(client.send_goal_async(goal), 10.0, f'{what} goal acceptance')
    if not handle.accepted:
        raise RuntimeError(f'{what}: goal rejected by the action server')
    wrapped = wait_for(handle.get_result_async(), timeout, f'{what} result')
    return wrapped.status, wrapped.result


def spin_in_background(node) -> tuple[SingleThreadedExecutor, threading.Thread]:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def to_pose_msg(pose: Pose) -> PoseMsg:
    msg = PoseMsg()
    msg.position.x, msg.position.y, msg.position.z = (float(v) for v in pose.position)
    (msg.orientation.x, msg.orientation.y,
     msg.orientation.z, msg.orientation.w) = (float(v) for v in pose.orientation)
    return msg


def from_pose_msg(msg: PoseMsg) -> Pose:
    return Pose(
        (msg.position.x, msg.position.y, msg.position.z),
        (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
    )


class JointStateCache:
    """Latest position of every joint seen on /joint_states."""

    def __init__(self, node):
        self._positions: dict[str, float] = {}
        self._lock = threading.Lock()
        self._sub = node.create_subscription(JointState, '/joint_states', self._on_msg, 10)

    def _on_msg(self, msg: JointState) -> None:
        with self._lock:
            self._positions.update(zip(msg.name, msg.position))

    def position(self, joint: str, timeout: float = 5.0) -> float:
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            with self._lock:
                if joint in self._positions:
                    return self._positions[joint]
            if time.monotonic() > deadline:
                break
            time.sleep(0.02)
        raise RosTimeout(f'no /joint_states value for {joint}')
