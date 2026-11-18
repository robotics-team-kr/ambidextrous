"""The task setup: where things are and what counts as success.

These numbers mirror dualarm_description/worlds/pick_place.sdf
(test/test_scene.py checks the two stay in sync). Frame: robot "world" link.
"""
from __future__ import annotations

from dataclasses import dataclass

WORLD_FRAME = 'world'
GZ_WORLD_NAME = 'pick_place'

# Table: box centred at TABLE_CENTER; its top surface is at TABLE_TOP_Z.
TABLE_SIZE = (0.6, 1.0, 0.2)
TABLE_CENTER = (0.5, 0.0, 0.1)
TABLE_TOP_Z = TABLE_CENTER[2] + TABLE_SIZE[2] / 2.0

# The object (comparison condition: 5 cm cube).
CUBE_NAME = 'cube'
CUBE_SIZE = 0.05
CUBE_REST_Z = TABLE_TOP_Z + CUBE_SIZE / 2.0

# Case 2: uniformly random inside a 30 x 30 cm square around the Case 1 position.
RANDOM_REGION_SIZE = (0.30, 0.30)

# A placed cube counts as a success if it lies flat within these tolerances of the target.
PLACE_TOLERANCE_XY = 0.03
PLACE_TOLERANCE_Z = 0.01


@dataclass(frozen=True)
class ArmScene:
    """Everything that depends on which arm does the picking."""

    side: str                          # 'left' or 'right'
    group: str                         # MoveIt planning group
    flange_link: str                   # tip link of that group
    arm_joints: tuple[str, ...]
    finger_joint: str
    hand_action: str                   # FollowJointTrajectory action of the hand
    fixed_cube_xy: tuple[float, float]   # Case 1 position
    place_xy: tuple[float, float]        # where the cube must end up

    @property
    def region_bounds(self) -> tuple[float, float, float, float]:
        """(x_min, x_max, y_min, y_max) of the Case 2 sampling square."""
        cx, cy = self.fixed_cube_xy
        sx, sy = RANDOM_REGION_SIZE
        return (cx - sx / 2.0, cx + sx / 2.0, cy - sy / 2.0, cy + sy / 2.0)


READY_JOINT_POSITIONS = (0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785)


def arm_scene(side: str) -> ArmScene:
    """Scene for the 'right' arm (base at y = -0.3) or its mirror image for 'left'."""
    if side not in ('left', 'right'):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    sign = -1.0 if side == 'right' else 1.0
    prefix = f'{side}_panda_'
    return ArmScene(
        side=side,
        group=f'{side}_panda_arm',
        flange_link=f'{prefix}link8',
        arm_joints=tuple(f'{prefix}joint{i}' for i in range(1, 8)),
        finger_joint=f'{prefix}finger_joint1',
        hand_action=f'/{side}_hand_controller/follow_joint_trajectory',
        fixed_cube_xy=(0.5, sign * 0.3),
        place_xy=(0.5, 0.0),
    )
