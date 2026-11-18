"""The simulated world, the robot model, the MoveIt config and scene.py must agree.

These read the files in the sibling packages directly from the source tree, so
they are skipped when the package is tested from an install without the sources.
"""
import math
import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from dualarm_pick_place import scene
from dualarm_pick_place.grasp import FINGER_OPEN_POSITION, FLANGE_TO_HAND_YAW, TCP_OFFSET

SRC = Path(__file__).resolve().parents[2]
WORLD = SRC / 'dualarm_description' / 'worlds' / 'pick_place.sdf'
ARM_XACRO = SRC / 'dualarm_description' / 'urdf' / 'panda_arm.xacro'
CONTROL_XACRO = SRC / 'dualarm_description' / 'urdf' / 'dual_panda.ros2_control.xacro'
CONTROLLERS = SRC / 'dualarm_description' / 'config' / 'ros2_controllers.yaml'
SRDF = SRC / 'dualarm_moveit_config' / 'config' / 'dual_panda.srdf'
MOVEIT_CONTROLLERS = SRC / 'dualarm_moveit_config' / 'config' / 'moveit_controllers.yaml'

XACRO_NS = '{http://www.ros.org/wiki/xacro}'


def floats(text):
    return tuple(float(v) for v in text.split())


@unittest.skipUnless(WORLD.exists(), 'source tree not available')
class TestWorldMatchesScene(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.world = ET.parse(WORLD).getroot().find('world')
        cls.models = {m.get('name'): m for m in cls.world.findall('model')}

    def test_world_name(self):
        self.assertEqual(self.world.get('name'), scene.GZ_WORLD_NAME)

    def test_table(self):
        table = self.models['table']
        self.assertEqual(floats(table.findtext('pose'))[:3], scene.TABLE_CENTER)
        self.assertEqual(floats(table.findtext('link/collision/geometry/box/size')), scene.TABLE_SIZE)

    def test_cube(self):
        cube = self.models[scene.CUBE_NAME]
        size = floats(cube.findtext('link/collision/geometry/box/size'))
        self.assertEqual(size, (scene.CUBE_SIZE,) * 3)
        x, y, z = floats(cube.findtext('pose'))[:3]
        self.assertEqual((x, y), scene.arm_scene('right').fixed_cube_xy)
        self.assertAlmostEqual(z, scene.CUBE_REST_Z)

    def test_cube_inertia_is_that_of_a_solid_cube(self):
        inertial = self.models[scene.CUBE_NAME].find('link/inertial')
        mass = float(inertial.findtext('mass'))
        expected = mass * scene.CUBE_SIZE ** 2 / 6.0
        for axis in ('ixx', 'iyy', 'izz'):
            self.assertAlmostEqual(float(inertial.findtext(f'inertia/{axis}')), expected, delta=expected * 0.01)

    def test_cube_publishes_its_pose(self):
        plugins = [p.get('name') for p in self.models[scene.CUBE_NAME].findall('plugin')]
        self.assertIn('ignition::gazebo::systems::PosePublisher', plugins)

    def test_place_marker_is_at_the_place_target_and_cannot_be_bumped(self):
        marker = self.models['place_target']
        self.assertEqual(floats(marker.findtext('pose'))[:2], scene.arm_scene('right').place_xy)
        self.assertIsNone(marker.find('link/collision'))


@unittest.skipUnless(ARM_XACRO.exists(), 'source tree not available')
class TestRobotModel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(ARM_XACRO).getroot()
        cls.properties = {p.get('name'): p.get('value') for p in cls.root.findall(f'{XACRO_NS}property')}
        arm_macro = next(m for m in cls.root.findall(f'{XACRO_NS}macro') if m.get('name') == 'panda_arm')
        cls.joints = {j.get('name'): j for j in arm_macro.findall('joint')}
        cls.links = arm_macro.findall(f'{XACRO_NS}panda_link')

    def test_tcp_offset_matches_grasp_code(self):
        self.assertEqual(float(self.properties['tcp_offset']), TCP_OFFSET)

    def test_hand_mount_angle_matches_grasp_code(self):
        rpy = self.joints['${prefix}panda_hand_joint'].find('origin').get('rpy')
        self.assertEqual(rpy, '0 0 ${-pi / 4}')
        self.assertAlmostEqual(FLANGE_TO_HAND_YAW, -math.pi / 4)

    def test_finger_travel_matches_gripper_code(self):
        limit = self.joints['${prefix}panda_finger_joint1'].find('limit')
        self.assertEqual(float(limit.get('upper')), FINGER_OPEN_POSITION)
        self.assertEqual(float(limit.get('lower')), 0.0)

    def test_open_gripper_fits_the_cube_with_clearance(self):
        self.assertGreater(2 * FINGER_OPEN_POSITION, scene.CUBE_SIZE + 0.02)

    def test_tcp_sits_at_the_centre_of_the_finger_pads(self):
        finger = next(m for m in self.root.findall(f'{XACRO_NS}macro') if m.get('name') == 'panda_finger')
        pad = finger.findall('link/collision')[-1]
        pad_z = floats(pad.find('origin').get('xyz').replace('${side * 0.00758}', '0'))[2]
        finger_joint_z = floats(self.joints['${prefix}panda_finger_joint1'].find('origin').get('xyz'))[2]
        self.assertAlmostEqual(finger_joint_z + pad_z, TCP_OFFSET, delta=0.001)

    def test_fingertips_clear_the_table_when_gripping_the_cube(self):
        finger = next(m for m in self.root.findall(f'{XACRO_NS}macro') if m.get('name') == 'panda_finger')
        pad = finger.findall('link/collision')[-1]
        pad_z = floats(pad.find('origin').get('xyz').replace('${side * 0.00758}', '0'))[2]
        pad_height = floats(pad.find('geometry/box').get('size'))[2]
        finger_joint_z = floats(self.joints['${prefix}panda_finger_joint1'].find('origin').get('xyz'))[2]
        tip_below_tcp = finger_joint_z + pad_z + pad_height / 2 - TCP_OFFSET
        self.assertLess(tip_below_tcp, scene.CUBE_SIZE / 2 - 0.005)

    def test_ready_pose_is_the_top_down_grasp_orientation(self):
        """Forward kinematics of the URDF at 'ready' vs. the grasp code's conventions.

        Independent check that yaw 0 in grasp.py really is how the hand hangs in the
        ready pose, and that tcp_to_flange inverts the URDF's flange -> TCP chain.
        """
        from dualarm_pick_place.geometry import Pose, quat_from_axis_angle, quat_multiply, rotate_vector
        from dualarm_pick_place.grasp import tcp_to_flange, top_down_orientation

        def evaluate(text):
            return [eval(part.replace('${', '').replace('}', ''), {'pi': math.pi})   # noqa: S307
                    for part in re.findall(r'\$\{[^}]*\}|[^\s]+', text)]

        position, orientation = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        flange = None
        chain = [f'panda_joint{i}' for i in range(1, 9)] + ['panda_hand_joint', 'panda_hand_tcp_joint']
        angles = dict(zip(chain, scene.READY_JOINT_POSITIONS))
        for name in chain:
            origin = self.joints['${prefix}' + name].find('origin')
            xyz = evaluate(origin.get('xyz').replace('${tcp_offset}', self.properties['tcp_offset']))
            roll, pitch, yaw = evaluate(origin.get('rpy'))
            position = tuple(p + d for p, d in zip(position, rotate_vector(orientation, xyz)))
            for axis, angle in (((0, 0, 1), yaw), ((0, 1, 0), pitch), ((1, 0, 0), roll),
                                ((0, 0, 1), angles.get(name, 0.0))):
                orientation = quat_multiply(orientation, quat_from_axis_angle(axis, angle))
            if name == 'panda_joint8':
                flange = Pose(position, orientation)
        tcp = Pose(position, orientation)

        # The hand hangs straight down, x forward, fingers along world y (1e-3: 0.785 vs pi/4).
        expected = top_down_orientation(0.0)
        for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            for a, e in zip(rotate_vector(tcp.orientation, axis), rotate_vector(expected, axis)):
                self.assertAlmostEqual(a, e, delta=2e-3)
        self.assertAlmostEqual(tcp.position[1], 0.0, delta=1e-3)
        self.assertGreater(tcp.position[0], 0.25)

        # tcp_to_flange(TCP from FK) must reproduce the flange from FK exactly.
        computed = tcp_to_flange(tcp)
        for a, e in zip(computed.position, flange.position):
            self.assertAlmostEqual(a, e, places=9)
        for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            for a, e in zip(rotate_vector(computed.orientation, axis), rotate_vector(flange.orientation, axis)):
                self.assertAlmostEqual(a, e, places=9)

    def test_link_inertias_are_physically_valid(self):
        self.assertEqual(len(self.links), 9)   # link0..link7 + hand
        for link in self.links:
            ixx, iyy, izz, ixy, ixz, iyz = (float(link.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'))
            name = link.get('name')
            self.assertGreater(float(link.get('mass')), 0.0, name)
            # Positive definite (Sylvester's criterion).
            self.assertGreater(ixx, 0.0, name)
            self.assertGreater(ixx * iyy - ixy ** 2, 0.0, name)
            det = (ixx * (iyy * izz - iyz ** 2) - ixy * (ixy * izz - iyz * ixz)
                   + ixz * (ixy * iyz - iyy * ixz))
            self.assertGreater(det, 0.0, name)


@unittest.skipUnless(SRDF.exists(), 'source tree not available')
class TestNamesAgreeAcrossPackages(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        import yaml
        cls.srdf = ET.parse(SRDF).getroot()
        cls.controllers = yaml.safe_load(CONTROLLERS.read_text())
        cls.moveit_controllers = yaml.safe_load(MOVEIT_CONTROLLERS.read_text())
        arm_xacro = ARM_XACRO.read_text()
        cls.link_names = set(re.findall(r'name="\$\{prefix\}(panda_\w+)"', arm_xacro))

    def test_srdf_groups_and_ready_state(self):
        for side in ('left', 'right'):
            arm = scene.arm_scene(side)
            group = next(g for g in self.srdf.findall('group') if g.get('name') == arm.group)
            self.assertEqual(group.find('chain').get('tip_link'), arm.flange_link)
            ready = next(s for s in self.srdf.findall('group_state')
                         if s.get('group') == arm.group and s.get('name') == 'ready')
            values = {j.get('name'): float(j.get('value')) for j in ready.findall('joint')}
            self.assertEqual(values, dict(zip(arm.arm_joints, scene.READY_JOINT_POSITIONS)))

    def test_srdf_only_names_links_that_exist(self):
        for pair in self.srdf.findall('disable_collisions'):
            for key in ('link1', 'link2'):
                side, _, rest = pair.get(key).partition('_')
                self.assertIn(side, ('left', 'right'))
                self.assertIn(rest, self.link_names, pair.get(key))

    def test_srdf_keeps_arm_to_arm_collision_checking(self):
        for pair in self.srdf.findall('disable_collisions'):
            self.assertEqual(pair.get('link1').split('_')[0], pair.get('link2').split('_')[0])

    def test_gazebo_start_pose_is_the_ready_pose(self):
        root = ET.parse(CONTROL_XACRO).getroot()
        macro = next(m for m in root.findall(f'{XACRO_NS}macro') if m.get('name') == 'panda_arm_interfaces')
        initial = [float(j.get('initial')) for j in macro.findall(f'{XACRO_NS}arm_joint')]
        self.assertEqual(tuple(initial), scene.READY_JOINT_POSITIONS)

    def test_controllers_cover_the_joints_the_code_commands(self):
        for side in ('left', 'right'):
            arm = scene.arm_scene(side)
            self.assertEqual(tuple(self.controllers[f'{side}_arm_controller']['ros__parameters']['joints']),
                             arm.arm_joints)
            self.assertEqual(self.controllers[f'{side}_hand_controller']['ros__parameters']['joints'],
                             [arm.finger_joint])
            self.assertEqual(arm.hand_action, f'/{side}_hand_controller/follow_joint_trajectory')

    def test_moveit_controllers_match_ros2_control(self):
        manager = self.moveit_controllers['moveit_simple_controller_manager']
        spawned = set(self.controllers['controller_manager']['ros__parameters']) - {'update_rate', 'joint_state_broadcaster'}
        self.assertEqual(set(manager['controller_names']), spawned)
        for name in manager['controller_names']:
            self.assertEqual(manager[name]['joints'], self.controllers[name]['ros__parameters']['joints'])


if __name__ == '__main__':
    unittest.main()
