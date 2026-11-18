"""Step 5: the pick & place sequence.

    open -> approach (접근) -> descend (하강) -> grasp (집기) -> lift (들기)
         -> transfer (옮기기) -> lower -> release (놓기) -> retreat

Each stage either finishes or stops the sequence, and the result names the stage
that failed so a failed trial can be diagnosed from the log alone.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from . import scene
from .geometry import Pose
from .grasp import GraspError, plan_top_down_grasp, tcp_to_flange
from .gripper import Gripper
from .moveit_client import MotionError, MoveItClient


@dataclass(frozen=True)
class PickPlaceConfig:
    approach_height: float = 0.12   # pregrasp height above the object centre [m]
    lift_height: float = 0.15       # carry height above the object centre [m]
    place_clearance: float = 0.003  # release this far above the surface [m]
    preferred_yaw: float = 0.0      # gripper heading to stay close to [rad]


@dataclass(frozen=True)
class PickPlaceResult:
    completed: bool        # every stage ran without error
    failed_stage: str      # '' if completed
    message: str
    duration: float        # wall-clock seconds


class StageFailed(RuntimeError):
    """A stage ran but did not achieve its purpose (e.g. closed on nothing)."""


class PickPlace:

    def __init__(self, arm: scene.ArmScene, moveit: MoveItClient, gripper: Gripper,
                 logger, config: PickPlaceConfig = PickPlaceConfig()):
        self._arm = arm
        self._moveit = moveit
        self._gripper = gripper
        self._log = logger
        self._config = config

    def setup_planning_scene(self) -> None:
        """Register the table with MoveIt so arm motions are planned around it."""
        self._moveit.add_box('table', scene.TABLE_SIZE, Pose(scene.TABLE_CENTER))

    def go_home(self) -> None:
        """Open the hand and return the arm to the ready pose."""
        self._gripper.open()
        self._moveit.move_to_joints(self._arm.group, self._arm.arm_joints,
                                    scene.READY_JOINT_POSITIONS)

    def run(self, object_pose: Pose, place_xy: tuple[float, float]) -> PickPlaceResult:
        """Pick the object at object_pose and put it down at place_xy (same surface)."""
        started = time.monotonic()
        config = self._config
        try:
            grasp = plan_top_down_grasp(
                object_pose, approach_height=config.approach_height,
                lift_height=config.lift_height, preferred_yaw=config.preferred_yaw)
        except GraspError as error:
            return PickPlaceResult(False, 'plan_grasp', str(error), time.monotonic() - started)

        surface_z = object_pose.position[2]
        orientation = grasp.grasp.orientation
        above_place = Pose((*place_xy, surface_z + config.lift_height), orientation)
        place = Pose((*place_xy, surface_z + config.place_clearance), orientation)

        stages = [
            ('open', self._gripper.open),
            ('approach', lambda: self._move_free(grasp.pregrasp)),
            ('descend', lambda: self._move_linear(grasp.grasp)),
            ('grasp', self._close_on_object),
            ('lift', lambda: self._move_linear(grasp.lift)),
            ('check_hold', self._require_holding),
            ('transfer', lambda: self._transfer(above_place)),
            ('lower', lambda: self._move_linear(place)),
            ('release', self._gripper.open),
            ('retreat', lambda: self._move_linear(above_place)),
        ]
        for name, action in stages:
            self._log.info(f'[{self._arm.side}] {name}')
            try:
                action()
            except (RuntimeError, TimeoutError) as error:   # MotionError, StageFailed, RosTimeout
                self._log.error(f'[{self._arm.side}] {name} failed: {error}')
                return PickPlaceResult(False, name, str(error), time.monotonic() - started)
        return PickPlaceResult(True, '', '', time.monotonic() - started)

    # ---- stages -------------------------------------------------------------

    def _move_free(self, tcp_pose: Pose) -> None:
        self._moveit.move_to_pose(self._arm.group, self._arm.flange_link, tcp_to_flange(tcp_pose))

    def _move_linear(self, tcp_pose: Pose) -> None:
        self._moveit.move_linear(self._arm.group, self._arm.flange_link, tcp_to_flange(tcp_pose))

    def _transfer(self, tcp_pose: Pose) -> None:
        # Carry in a straight line at constant height so the object cannot be swung
        # into the table; fall back to a planned motion if the line is not feasible.
        try:
            self._move_linear(tcp_pose)
        except MotionError as error:
            self._log.warn(f'straight transfer not possible ({error}); planning a free motion')
            self._move_free(tcp_pose)

    def _close_on_object(self) -> None:
        self._gripper.close()
        self._require_holding()

    def _require_holding(self) -> None:
        if not self._gripper.is_holding():
            raise StageFailed('fingers closed completely: nothing is held '
                              f'(finger position {self._gripper.finger_position() * 1000:.1f} mm)')
