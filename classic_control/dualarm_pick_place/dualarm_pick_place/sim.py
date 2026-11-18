"""시뮬레이터에서만 쓰는 기능: 큐브를 원하는 곳으로 순간이동시키기.

실험(evaluate.py)에서 시행마다 큐브를 시작 위치에 놓을 때만 씁니다.
집어 옮기는 동작 자체는 이 파일을 쓰지 않습니다.
"""
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from . import scene
from .ros_helpers import to_pose_msg, wait_for


class SimWorld:

    def __init__(self, node):
        self._service = f'/world/{scene.GZ_WORLD_NAME}/set_pose'
        self._set_pose = node.create_client(SetEntityPose, self._service)

    def wait_until_ready(self, timeout=60.0):
        if not self._set_pose.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {self._service} not found (is the ros_gz bridge running?)')

    def set_object_pose(self, name, pose):
        """이름이 name 인 모델을 pose 로 옮깁니다."""
        request = SetEntityPose.Request()
        request.entity = Entity(name=name, type=Entity.MODEL)
        request.pose = to_pose_msg(pose)
        response = wait_for(self._set_pose.call_async(request), 10.0, 'set_pose')
        if not response.success:
            raise RuntimeError(f'Gazebo refused to move {name!r}')
