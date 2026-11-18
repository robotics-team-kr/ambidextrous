"""Step 6 helpers: where to put the cube for each trial and how to judge the outcome.

Plain Python so the rules of the experiment can be unit-tested without a simulator.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

from . import scene
from .geometry import Pose, box_tilt, quat_from_axis_angle

# Keep the whole cube inside the sampling square, not just its centre.
_EDGE_MARGIN = scene.CUBE_SIZE / 2.0


def cube_start_pose(case: int, arm: scene.ArmScene, rng: random.Random,
                    random_yaw: bool = False) -> Pose:
    """Start pose of the cube for one trial.

    Case 1: always the fixed position. Case 2: uniformly random in the 30 x 30 cm square.
    """
    if case == 1:
        x, y = arm.fixed_cube_xy
        yaw = 0.0
    elif case == 2:
        x_min, x_max, y_min, y_max = arm.region_bounds
        x = rng.uniform(x_min + _EDGE_MARGIN, x_max - _EDGE_MARGIN)
        y = rng.uniform(y_min + _EDGE_MARGIN, y_max - _EDGE_MARGIN)
        yaw = rng.uniform(-math.pi, math.pi) if random_yaw else 0.0
    else:
        raise ValueError(f'case must be 1 or 2, got {case}')
    return Pose((x, y, scene.CUBE_REST_Z), quat_from_axis_angle((0.0, 0.0, 1.0), yaw))


@dataclass(frozen=True)
class PlaceCheck:
    success: bool
    xy_error: float      # horizontal distance from the place target [m]
    z_error: float       # height above/below resting on the table [m]
    reason: str          # empty on success


def check_placement(final_pose: Pose, place_xy: tuple[float, float],
                    tol_xy: float = scene.PLACE_TOLERANCE_XY,
                    tol_z: float = scene.PLACE_TOLERANCE_Z) -> PlaceCheck:
    """Did the cube end up resting flat on the table at the place target?"""
    x, y, z = final_pose.position
    xy_error = math.hypot(x - place_xy[0], y - place_xy[1])
    z_error = z - scene.CUBE_REST_Z
    flat = box_tilt(final_pose.orientation) < math.radians(5.0)

    if xy_error > tol_xy:
        reason = f'{xy_error * 100:.1f} cm from target (limit {tol_xy * 100:.1f} cm)'
    elif abs(z_error) > tol_z:
        reason = f'not resting on the table (z off by {z_error * 100:.1f} cm)'
    elif not flat:
        reason = 'not lying flat'
    else:
        reason = ''
    return PlaceCheck(success=not reason, xy_error=xy_error, z_error=z_error, reason=reason)


def summarize(successes: int, trials: int, target_rate: float = 0.8) -> str:
    rate = successes / trials if trials else 0.0
    verdict = 'PASS' if trials and rate >= target_rate else 'FAIL'
    return f'{successes}/{trials} succeeded ({rate:.0%}); target {target_rate:.0%} -> {verdict}'
