"""Everything the pick & place needs: Gazebo + controllers + MoveIt.

    ros2 launch pick_place_damsssss sim.launch.py
    ros2 launch pick_place_damsssss sim.launch.py headless:=true rviz:=true

Then, in another terminal:
    ros2 run pick_place_damsssss pick_place
    ros2 run pick_place_damsssss evaluate --ros-args -p case:=1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('dualarm_description'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('dualarm_moveit_config'), 'launch', 'move_group.launch.py'])),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo server only (no GUI).'),
        DeclareLaunchArgument('rviz', default_value='false', description='Also start RViz.'),
        gazebo,
        move_group,
    ])
