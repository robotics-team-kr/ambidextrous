"""실행 프로그램(pick_place_node.py, evaluate.py)이 공통으로 쓰는 준비 작업.

ROS 노드를 하나 만들고, 필요한 부품(MoveIt, 그리퍼, 큐브 위치, 시뮬레이터)을 조립합니다.
"""
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor

from . import scene
from .gripper import Gripper
from .moveit_client import MoveItClient
from .object_pose import SimObjectPoseSource
from .pick_place import PickPlace
from .sim import SimWorld


class App:

    def __init__(self, node_name):
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.log = self.node.get_logger()

        # 어느 팔을 쓸지는 실행할 때 정합니다:  --ros-args -p arm:=left
        self.arm = scene.arm_scene(self.node.declare_parameter('arm', 'right').value)

        self.moveit = MoveItClient(self.node)
        self.gripper = Gripper(self.node, self.arm.hand_action, self.arm.finger_joint)
        self.objects = SimObjectPoseSource(self.node)   # 카메라를 쓰게 되면 이 줄만 바꿉니다
        self.sim = SimWorld(self.node)
        self.pick_place = PickPlace(self.arm, self.moveit, self.gripper, self.log)

        # 배경 스레드에서 ROS 메시지를 계속 받습니다. 덕분에 메인 코드는
        # "보내고 기다린다"를 순서대로 쓰기만 하면 됩니다.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def wait_until_ready(self):
        """MoveIt, 그리퍼, 시뮬레이터가 모두 켜질 때까지 기다립니다."""
        self.log.info('waiting for MoveIt, the gripper controller and the simulator...')
        self.moveit.wait_until_ready()
        self.gripper.wait_until_ready()
        self.sim.wait_until_ready()
        self.objects.latest(timeout=60.0)
        self.pick_place.setup_planning_scene()
        self.log.info('ready')

    def shutdown(self):
        # 배경 스레드를 먼저 멈춘 뒤 노드를 없앱니다. 순서가 바뀌면 Humble 에서 프로그램이 죽습니다.
        self._executor.shutdown(timeout_sec=1.0)
        self._spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()
