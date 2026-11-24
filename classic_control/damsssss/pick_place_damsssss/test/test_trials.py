import math
import random
import unittest

from pick_place_damsssss import scene
from pick_place_damsssss.geometry import Pose, quat_from_axis_angle
from pick_place_damsssss.trials import check_placement, cube_start_pose, summarize

RIGHT = scene.ArmScene('right')
LEFT = scene.ArmScene('left')


class TestArmScene(unittest.TestCase):

    def test_left_is_mirror_of_right(self):
        self.assertEqual(RIGHT.fixed_cube_xy, (0.5, -0.3))
        self.assertEqual(LEFT.fixed_cube_xy, (0.5, 0.3))
        self.assertEqual(RIGHT.place_xy, LEFT.place_xy)

    def test_names(self):
        self.assertEqual(RIGHT.group, 'right_panda_arm')
        self.assertEqual(RIGHT.flange_link, 'right_panda_link8')
        self.assertEqual(RIGHT.arm_joints[0], 'right_panda_joint1')
        self.assertEqual(len(RIGHT.arm_joints), len(scene.READY_JOINT_POSITIONS))

    def test_region_is_30cm_square_on_the_table(self):
        x_min, x_max, y_min, y_max = RIGHT.region_bounds
        self.assertAlmostEqual(x_max - x_min, 0.30)
        self.assertAlmostEqual(y_max - y_min, 0.30)
        table_x = (scene.TABLE_CENTER[0] - scene.TABLE_SIZE[0] / 2, scene.TABLE_CENTER[0] + scene.TABLE_SIZE[0] / 2)
        table_y = (scene.TABLE_CENTER[1] - scene.TABLE_SIZE[1] / 2, scene.TABLE_CENTER[1] + scene.TABLE_SIZE[1] / 2)
        self.assertGreaterEqual(x_min, table_x[0])
        self.assertLessEqual(x_max, table_x[1])
        self.assertGreaterEqual(y_min, table_y[0])
        self.assertLessEqual(y_max, table_y[1])

    def test_place_target_is_outside_the_pick_region(self):
        # Otherwise a cube could start already "placed".
        for arm in (RIGHT, LEFT):
            x_min, x_max, y_min, y_max = arm.region_bounds
            px, py = arm.place_xy
            dx = max(x_min - px, 0.0, px - x_max)
            dy = max(y_min - py, 0.0, py - y_max)
            self.assertGreater(math.hypot(dx, dy), scene.PLACE_TOLERANCE_XY + scene.CUBE_SIZE)

    def test_rejects_unknown_side(self):
        with self.assertRaises(ValueError):
            scene.ArmScene('middle')


class TestCubeStartPose(unittest.TestCase):

    def test_case1_is_always_the_fixed_position(self):
        rng = random.Random(0)
        for _ in range(5):
            pose = cube_start_pose(1, RIGHT, rng)
            self.assertEqual(pose.position, (0.5, -0.3, scene.CUBE_REST_Z))

    def test_case2_keeps_whole_cube_inside_region(self):
        rng = random.Random(1)
        x_min, x_max, y_min, y_max = RIGHT.region_bounds
        half = scene.CUBE_SIZE / 2
        xs, ys = [], []
        for _ in range(2000):
            x, y, z = cube_start_pose(2, RIGHT, rng).position
            self.assertGreaterEqual(x - half, x_min)
            self.assertLessEqual(x + half, x_max)
            self.assertGreaterEqual(y - half, y_min)
            self.assertLessEqual(y + half, y_max)
            self.assertAlmostEqual(z, scene.CUBE_REST_Z)
            xs.append(x)
            ys.append(y)
        # Actually spread over the square, not stuck near the centre.
        self.assertGreater(max(xs) - min(xs), 0.24)
        self.assertGreater(max(ys) - min(ys), 0.24)

    def test_case2_is_reproducible_from_seed(self):
        a = [cube_start_pose(2, RIGHT, random.Random(7)).position for _ in range(1)]
        b = [cube_start_pose(2, RIGHT, random.Random(7)).position for _ in range(1)]
        self.assertEqual(a, b)

    def test_case2_differs_between_trials(self):
        rng = random.Random(3)
        self.assertNotEqual(cube_start_pose(2, RIGHT, rng).position, cube_start_pose(2, RIGHT, rng).position)

    def test_invalid_case(self):
        with self.assertRaises(ValueError):
            cube_start_pose(3, RIGHT, random.Random(0))


class TestCheckPlacement(unittest.TestCase):

    def test_cube_on_target_succeeds(self):
        check = check_placement(Pose((0.51, -0.01, scene.CUBE_REST_Z)), RIGHT.place_xy)
        self.assertTrue(check.success)
        self.assertEqual(check.reason, '')
        self.assertAlmostEqual(check.xy_error, math.hypot(0.01, 0.01))

    def test_cube_left_at_start_fails(self):
        check = check_placement(Pose((0.5, -0.3, scene.CUBE_REST_Z)), RIGHT.place_xy)
        self.assertFalse(check.success)
        self.assertIn('from target', check.reason)

    def test_just_outside_xy_tolerance_fails(self):
        x = RIGHT.place_xy[0] + scene.PLACE_TOLERANCE_XY + 0.002
        self.assertFalse(check_placement(Pose((x, 0.0, scene.CUBE_REST_Z)), RIGHT.place_xy).success)

    def test_cube_still_in_the_air_fails(self):
        check = check_placement(Pose((0.5, 0.0, scene.CUBE_REST_Z + 0.1)), RIGHT.place_xy)
        self.assertFalse(check.success)
        self.assertIn('not resting', check.reason)

    def test_cube_on_the_floor_fails(self):
        self.assertFalse(check_placement(Pose((0.5, 0.0, scene.CUBE_SIZE / 2)), RIGHT.place_xy).success)

    def test_cube_on_an_edge_fails(self):
        tipped = quat_from_axis_angle((1.0, 0.0, 0.0), math.radians(45))
        check = check_placement(Pose((0.5, 0.0, scene.CUBE_REST_Z + 0.005), tipped), RIGHT.place_xy)
        self.assertFalse(check.success)
        self.assertEqual(check.reason, 'not lying flat')

    def test_any_face_down_and_any_yaw_is_fine(self):
        on_side = quat_from_axis_angle((0.0, 1.0, 0.0), math.pi / 2)
        turned = quat_from_axis_angle((0.0, 0.0, 1.0), 1.0)
        for q in (on_side, turned):
            self.assertTrue(check_placement(Pose((0.5, 0.0, scene.CUBE_REST_Z), q), RIGHT.place_xy).success)


class TestSummarize(unittest.TestCase):

    def test_eight_of_ten_passes(self):
        self.assertIn('PASS', summarize(8, 10))
        self.assertIn('8/10', summarize(8, 10))

    def test_seven_of_ten_fails(self):
        self.assertIn('FAIL', summarize(7, 10))

    def test_no_trials_is_not_a_pass(self):
        self.assertIn('FAIL', summarize(0, 0))


if __name__ == '__main__':
    unittest.main()
