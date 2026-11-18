"""Step 1: bring the dual-arm robot up in Gazebo with its controllers.

    ros2 launch dualarm_description gazebo.launch.py
    ros2 launch dualarm_description gazebo.launch.py headless:=true   # no GUI
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

WORLD_NAME = 'pick_place'

CONTROLLERS = [
    'joint_state_broadcaster',
    'left_arm_controller',
    'right_arm_controller',
    'left_hand_controller',
    'right_hand_controller',
]


def generate_launch_description():
    description_share = FindPackageShare('dualarm_description')
    world = PathJoinSubstitution([description_share, 'worlds', 'pick_place.sdf'])
    headless = LaunchConfiguration('headless')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([description_share, 'urdf', 'dual_panda.urdf.xacro']),
                ' position_gain:=', LaunchConfiguration('position_gain'),
            ]),
            value_type=str,
        )
    }

    # Gazebo resolves package://<pkg>/... mesh URIs by looking for <pkg> under its
    # resource path, so it needs the "share" directory that holds the Panda meshes.
    # Fortress (Humble) reads IGN_GAZEBO_RESOURCE_PATH; newer Gazebo reads GZ_SIM_RESOURCE_PATH.
    panda_share_parent = os.path.dirname(
        get_package_share_directory('moveit_resources_panda_description'))
    resource_paths = [
        AppendEnvironmentVariable(name, panda_share_parent)
        for name in ('IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH')
    ]

    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -v 2 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': ['-r -s -v 2 ', world], 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-world', WORLD_NAME, '-topic', 'robot_description', '-name', 'dual_panda'],
    )

    # /clock, the cube pose ("the sim tells us where the object is") and the service
    # used to move the cube between trials.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/cube/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',
            f'/world/{WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
    )

    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[*CONTROLLERS, '--controller-manager-timeout', '120'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo server only (no GUI).'),
        DeclareLaunchArgument('position_gain', default_value='0.1',
                              description='gz_ros2_control position_proportional_gain.'),
        *resource_paths,
        gazebo_gui,
        gazebo_headless,
        bridge,
        robot_state_publisher,
        spawn_robot,
        # The controller manager lives inside the Gazebo plugin, so it only exists
        # once the robot has been spawned.
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[spawn_controllers])),
    ])
