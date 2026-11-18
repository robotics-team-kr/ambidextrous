"""Minimal pose / quaternion math with no ROS or numpy dependency.

Quaternions are (x, y, z, w), the same order as geometry_msgs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]

IDENTITY_QUAT: Quat = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Pose:
    position: Vec3
    orientation: Quat = IDENTITY_QUAT


def quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_from_axis_angle(axis: Vec3, angle: float) -> Quat:
    norm = math.sqrt(sum(c * c for c in axis))
    s = math.sin(angle / 2.0) / norm
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2.0))


def rotate_vector(q: Quat, v: Vec3) -> Vec3:
    """Rotate v by q (v' = q v q*)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


_BODY_AXES: tuple[Vec3, ...] = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def box_tilt(q: Quat) -> float:
    """How far a box is from resting on one of its faces [rad].

    0 means some body axis is exactly vertical, whichever face is down.
    """
    most_vertical = max(abs(rotate_vector(q, axis)[2]) for axis in _BODY_AXES)
    return math.acos(min(1.0, most_vertical))


def box_face_yaw(q: Quat) -> float:
    """Heading of one of the box's side faces: angle of a horizontal face normal about world z.

    For a box resting on a face, two body axes are horizontal; either works because
    the result is only meaningful modulo 90 degrees.
    """
    most_horizontal = min((rotate_vector(q, axis) for axis in _BODY_AXES), key=lambda v: abs(v[2]))
    return math.atan2(most_horizontal[1], most_horizontal[0])


def wrap_to(angle: float, period: float) -> float:
    """Wrap angle into [-period/2, period/2)."""
    return (angle + period / 2.0) % period - period / 2.0
