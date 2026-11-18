"""Wires the pieces together for the command-line nodes."""
from __future__ import annotations

import rclpy

from . import scene
from .gripper import Gripper
from .moveit_client import MoveItClient
from .object_pose import SimObjectPoseSource
from .pick_place import PickPlace
from .ros_helpers import JointStateCache, spin_in_background
from .sim import SimWorld


class App:
    """One node plus everything a pick & place run needs."""

    def __init__(self, node_name: str):
        rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.log = self.node.get_logger()
        self.arm = scene.arm_scene(self.node.declare_parameter('arm', 'right').value)

        joint_states = JointStateCache(self.node)
        self.moveit = MoveItClient(self.node)
        self.gripper = Gripper(self.node, self.arm.hand_action, self.arm.finger_joint, joint_states)
        self.objects = SimObjectPoseSource(self.node)   # swap for a camera-based source later
        self.sim = SimWorld(self.node)
        self.pick_place = PickPlace(self.arm, self.moveit, self.gripper, self.log)

        self._executor, self._spin_thread = spin_in_background(self.node)

    def wait_until_ready(self) -> None:
        self.log.info('waiting for MoveIt, the gripper controller and the simulator...')
        self.moveit.wait_until_ready()
        self.gripper.wait_until_ready()
        self.sim.wait_until_ready()
        self.objects.latest(timeout=60.0)
        self.pick_place.setup_planning_scene()
        self.log.info('ready')

    def shutdown(self) -> None:
        # Stop the spin thread before tearing the node down: destroying a node that
        # is still being spun aborts the process on Humble.
        self._executor.shutdown(timeout_sec=1.0)
        self._spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()
