"""Step 4: from an object pose to the gripper poses that pick it up.

Strategy: top-down grasp. The gripper points straight down, its fingers close
across two opposite faces of the cube, and the point between the finger pads
(the TCP) ends up at the cube centre.

Everything here is plain math so it can be unit-tested without ROS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import (
    Pose,
    Quat,
    box_face_yaw,
    box_tilt,
    quat_from_axis_angle,
    quat_multiply,
    rotate_vector,
    wrap_to,
)

# Panda hand geometry (see dualarm_description/urdf/panda_arm.xacro).
TCP_OFFSET = 0.1034              # flange/hand origin -> point between the finger pads, along hand z
FLANGE_TO_HAND_YAW = -math.pi / 4  # the hand is mounted rotated about the flange z-axis
FINGER_OPEN_POSITION = 0.04      # each finger 4 cm from the centre -> 8 cm gap
FINGER_CLOSED_POSITION = 0.0

# A cube looks the same every 90 degrees about the vertical.
CUBE_YAW_SYMMETRY = math.pi / 2


class GraspError(ValueError):
    """The object pose cannot be grasped with the top-down strategy."""


@dataclass(frozen=True)
class GraspPlan:
    """TCP poses for the pick, all with the same top-down orientation."""

    pregrasp: Pose   # above the object, fingers clear of it
    grasp: Pose      # TCP at the object centre
    lift: Pose       # straight up after closing
    yaw: float       # gripper rotation about the vertical


def top_down_orientation(yaw: float) -> Quat:
    """Gripper pointing down (hand z = -world z), rotated by yaw about the vertical.

    At yaw = 0 the hand's x-axis is world +x and the fingers open/close along world y,
    which is the orientation of the 'ready' pose.
    """
    flip = quat_from_axis_angle((1.0, 0.0, 0.0), math.pi)
    spin = quat_from_axis_angle((0.0, 0.0, 1.0), yaw)
    return quat_multiply(spin, flip)


def choose_grasp_yaw(object_yaw: float, preferred_yaw: float = 0.0,
                     symmetry: float = CUBE_YAW_SYMMETRY) -> float:
    """Pick the gripper yaw that squares up with the object's faces.

    Any yaw equal to the object's yaw modulo `symmetry` grips flat faces; of those,
    return the one closest to preferred_yaw so the wrist turns as little as possible.
    """
    return preferred_yaw + wrap_to(object_yaw - preferred_yaw, symmetry)


def plan_top_down_grasp(object_pose: Pose, *, approach_height: float = 0.12,
                        lift_height: float = 0.15, preferred_yaw: float = 0.0,
                        max_tilt: float = math.radians(10.0)) -> GraspPlan:
    """Compute pregrasp / grasp / lift TCP poses for a cube resting on a surface."""
    tilt = box_tilt(object_pose.orientation)
    if tilt > max_tilt:
        # No face is level: the cube is leaning on something.
        raise GraspError(f'object is tilted {math.degrees(tilt):.1f} deg; top-down grasp needs it flat')

    yaw = choose_grasp_yaw(box_face_yaw(object_pose.orientation), preferred_yaw)
    orientation = top_down_orientation(yaw)
    x, y, z = object_pose.position
    return GraspPlan(
        pregrasp=Pose((x, y, z + approach_height), orientation),
        grasp=Pose((x, y, z), orientation),
        lift=Pose((x, y, z + lift_height), orientation),
        yaw=yaw,
    )


def tcp_to_flange(tcp_pose: Pose) -> Pose:
    """Convert a desired TCP pose into the pose of the arm's tip link (panda_link8).

    MoveIt plans for the planning group's tip link, so every TCP target goes through
    this before it is sent.
    """
    hand_z = rotate_vector(tcp_pose.orientation, (0.0, 0.0, 1.0))
    position = tuple(p - TCP_OFFSET * a for p, a in zip(tcp_pose.position, hand_z))
    undo_mount = quat_from_axis_angle((0.0, 0.0, 1.0), -FLANGE_TO_HAND_YAW)
    return Pose(position, quat_multiply(tcp_pose.orientation, undo_mount))
