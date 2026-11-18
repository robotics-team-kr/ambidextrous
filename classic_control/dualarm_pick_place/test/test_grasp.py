import math
import unittest

from dualarm_pick_place.geometry import (
    Pose,
    box_face_yaw,
    box_tilt,
    quat_from_axis_angle,
    quat_multiply,
    rotate_vector,
)
from dualarm_pick_place.grasp import (
    TCP_OFFSET,
    GraspError,
    choose_grasp_yaw,
    plan_top_down_grasp,
    tcp_to_flange,
    top_down_orientation,
)

Z = (0.0, 0.0, 1.0)


def yawed(yaw):
    return quat_from_axis_angle(Z, yaw)


class AssertVec(unittest.TestCase):

    def assert_vec(self, actual, expected, places=9):
        for a, e in zip(actual, expected, strict=True):
            self.assertAlmostEqual(a, e, places=places)


class TestGeometry(AssertVec):

    def test_rotate_vector_quarter_turn_about_z(self):
        self.assert_vec(rotate_vector(yawed(math.pi / 2), (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))

    def test_quat_multiply_composes_rotations(self):
        q = quat_multiply(yawed(0.3), yawed(0.4))
        self.assert_vec(q, yawed(0.7))

    def test_box_tilt_is_zero_on_any_face(self):
        on_side = quat_from_axis_angle((1.0, 0.0, 0.0), math.pi / 2)
        upside_down = quat_from_axis_angle((0.0, 1.0, 0.0), math.pi)
        for q in (yawed(0.7), on_side, upside_down):
            self.assertAlmostEqual(box_tilt(q), 0.0, places=6)

    def test_box_tilt_detects_leaning_box(self):
        leaning = quat_from_axis_angle((1.0, 0.0, 0.0), math.radians(30))
        self.assertAlmostEqual(box_tilt(leaning), math.radians(30), places=6)

    def test_box_face_yaw_of_box_on_its_side(self):
        # x-axis pointing up: the heading must come from an axis that is still horizontal.
        x_up = quat_from_axis_angle((0.0, 1.0, 0.0), -math.pi / 2)
        q = quat_multiply(yawed(0.2), x_up)
        yaw = box_face_yaw(q)
        self.assertAlmostEqual(math.remainder(yaw - 0.2, math.pi / 2), 0.0, places=6)


class TestGraspYaw(unittest.TestCase):

    def test_aligned_cube_keeps_preferred_yaw(self):
        self.assertAlmostEqual(choose_grasp_yaw(0.0), 0.0)

    def test_uses_ninety_degree_symmetry(self):
        # A cube turned 100 deg looks like one turned 10 deg.
        self.assertAlmostEqual(choose_grasp_yaw(math.radians(100)), math.radians(10))
        self.assertAlmostEqual(choose_grasp_yaw(math.radians(-170)), math.radians(10))

    def test_never_turns_more_than_45_degrees_from_preferred(self):
        for degrees in range(-180, 181, 7):
            yaw = choose_grasp_yaw(math.radians(degrees))
            self.assertLessEqual(abs(yaw), math.pi / 4 + 1e-9)
            # ...and is still square to the cube's faces.
            self.assertAlmostEqual(math.remainder(yaw - math.radians(degrees), math.pi / 2), 0.0)


class TestTopDownGrasp(AssertVec):

    def test_orientation_points_down_with_fingers_along_world_y_at_zero_yaw(self):
        q = top_down_orientation(0.0)
        self.assert_vec(rotate_vector(q, (0.0, 0.0, 1.0)), (0.0, 0.0, -1.0))   # approach axis
        self.assert_vec(rotate_vector(q, (1.0, 0.0, 0.0)), (1.0, 0.0, 0.0))
        self.assert_vec(rotate_vector(q, (0.0, 1.0, 0.0)), (0.0, -1.0, 0.0))   # finger axis

    def test_fingers_close_across_cube_faces(self):
        cube_yaw = math.radians(25)
        plan = plan_top_down_grasp(Pose((0.5, -0.3, 0.225), yawed(cube_yaw)))
        finger_axis = rotate_vector(plan.grasp.orientation, (0.0, 1.0, 0.0))
        self.assertAlmostEqual(finger_axis[2], 0.0)
        finger_heading = math.atan2(finger_axis[1], finger_axis[0])
        # Parallel to a face normal: equal to the cube yaw modulo 90 degrees.
        self.assertAlmostEqual(math.remainder(finger_heading - cube_yaw, math.pi / 2), 0.0)

    def test_waypoints_are_stacked_vertically_over_the_object(self):
        plan = plan_top_down_grasp(Pose((0.5, -0.3, 0.225)), approach_height=0.12, lift_height=0.15)
        self.assert_vec(plan.grasp.position, (0.5, -0.3, 0.225))
        self.assert_vec(plan.pregrasp.position, (0.5, -0.3, 0.345))
        self.assert_vec(plan.lift.position, (0.5, -0.3, 0.375))
        self.assertEqual(plan.pregrasp.orientation, plan.grasp.orientation)

    def test_rejects_leaning_object(self):
        leaning = quat_from_axis_angle((1.0, 0.0, 0.0), math.radians(30))
        with self.assertRaises(GraspError):
            plan_top_down_grasp(Pose((0.5, -0.3, 0.225), leaning))

    def test_accepts_cube_resting_on_another_face(self):
        on_side = quat_from_axis_angle((1.0, 0.0, 0.0), math.pi / 2)
        plan = plan_top_down_grasp(Pose((0.5, -0.3, 0.225), on_side))
        self.assert_vec(rotate_vector(plan.grasp.orientation, (0.0, 0.0, 1.0)), (0.0, 0.0, -1.0))


class TestTcpToFlange(AssertVec):

    def test_flange_is_tcp_offset_above_a_downward_tcp(self):
        tcp = Pose((0.5, -0.3, 0.225), top_down_orientation(0.4))
        flange = tcp_to_flange(tcp)
        self.assert_vec(flange.position, (0.5, -0.3, 0.225 + TCP_OFFSET))

    def test_matches_urdf_chain_flange_to_tcp(self):
        # URDF: link8 -[rpy 0 0 -pi/4]-> hand -[xyz 0 0 TCP_OFFSET]-> tcp.
        # Going forward along that chain from the computed flange must give back the TCP.
        tcp = Pose((0.42, -0.21, 0.3), top_down_orientation(-0.6))
        flange = tcp_to_flange(tcp)
        hand_q = quat_multiply(flange.orientation, quat_from_axis_angle(Z, -math.pi / 4))
        offset = rotate_vector(hand_q, (0.0, 0.0, TCP_OFFSET))
        self.assert_vec(tuple(p + o for p, o in zip(flange.position, offset)), tcp.position)
        self.assert_vec(hand_q, tcp.orientation)

    def test_ready_pose_orientation(self):
        # In the 'ready' pose the hand is top-down at yaw 0 and joint7 = +pi/4 cancels
        # the hand mount, so the flange is the hand frame turned +45 deg about its own z.
        flange = tcp_to_flange(Pose((0.3, 0.0, 0.5), top_down_orientation(0.0)))
        x_axis = rotate_vector(flange.orientation, (1.0, 0.0, 0.0))
        s = math.sqrt(0.5)
        self.assert_vec(x_axis, (s, -s, 0.0))


if __name__ == '__main__':
    unittest.main()
