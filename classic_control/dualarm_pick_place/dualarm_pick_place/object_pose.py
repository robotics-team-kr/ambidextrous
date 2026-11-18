"""Where is the object?  <- the one place to change when a camera replaces the simulator.

The pick & place code only ever calls get_object_pose(source). Today the source is
SimObjectPoseSource (Gazebo publishes the cube's true pose). To switch to a camera,
write another class with the same two methods (latest / wait_until_still) that returns
the pose estimated from images, and pass that instead. Nothing else needs to change.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Protocol

from geometry_msgs.msg import PoseStamped

from . import scene
from .geometry import Pose
from .ros_helpers import RosTimeout, from_pose_msg


class ObjectPoseSource(Protocol):
    """Anything that can report the object's pose in the robot's world frame."""

    def latest(self, timeout: float = 5.0) -> Pose:
        """Most recent pose, waiting for a new measurement to arrive."""

    def wait_until_still(self, timeout: float = 10.0) -> Pose:
        """Pose once the object has stopped moving."""


def get_object_pose(source: ObjectPoseSource, timeout: float = 10.0) -> Pose:
    """Pose of the object to pick, in the world frame, once it is at rest."""
    return source.wait_until_still(timeout)


class SimObjectPoseSource:
    """Ground-truth pose from Gazebo (PosePublisher plugin -> ros_gz_bridge).

    Gazebo's world origin coincides with the robot's "world" link, so the pose is
    used as-is.
    """

    def __init__(self, node, object_name: str = scene.CUBE_NAME):
        self._pose: Pose | None = None
        self._count = 0
        self._lock = threading.Lock()
        self._topic = f'/model/{object_name}/pose'
        self._sub = node.create_subscription(PoseStamped, self._topic, self._on_pose, 10)

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._pose = from_pose_msg(msg.pose)
            self._count += 1

    def latest(self, timeout: float = 5.0) -> Pose:
        # Wait for a message newer than this call so a stale pose is never returned
        # (e.g. right after the cube has been teleported for a new trial).
        with self._lock:
            seen = self._count
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._count > seen and self._pose is not None:
                    return self._pose
            time.sleep(0.01)
        raise RosTimeout(f'no object pose on {self._topic} (is the simulator running?)')

    def wait_until_still(self, timeout: float = 10.0, window: float = 0.5,
                         tolerance: float = 0.001) -> Pose:
        deadline = time.monotonic() + timeout
        previous = self.latest()
        while time.monotonic() < deadline:
            time.sleep(window)
            current = self.latest()
            if math.dist(previous.position, current.position) < tolerance:
                return current
            previous = current
        raise RosTimeout(f'object still moving after {timeout:.0f} s')
