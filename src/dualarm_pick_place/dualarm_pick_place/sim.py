"""Simulator-only helpers for the experiments (not used by the pick & place itself)."""
from __future__ import annotations

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from . import scene
from .geometry import Pose
from .ros_helpers import to_pose_msg, wait_for


class SimWorld:

    def __init__(self, node):
        self._service = f'/world/{scene.GZ_WORLD_NAME}/set_pose'
        self._set_pose = node.create_client(SetEntityPose, self._service)

    def wait_until_ready(self, timeout: float = 60.0) -> None:
        if not self._set_pose.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {self._service} not found (is the ros_gz bridge running?)')

    def set_object_pose(self, name: str, pose: Pose) -> None:
        """Teleport a model, e.g. put the cube at its start position for a new trial."""
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose = to_pose_msg(pose)
        response = wait_for(self._set_pose.call_async(request), 10.0, 'set_pose')
        if not response.success:
            raise RuntimeError(f'Gazebo refused to move {name!r}')
