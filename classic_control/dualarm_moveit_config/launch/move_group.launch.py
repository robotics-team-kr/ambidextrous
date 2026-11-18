"""Step 2: MoveIt for the simulated robot (expects gazebo.launch.py to be running).

    ros2 launch dualarm_moveit_config move_group.launch.py
    ros2 launch dualarm_moveit_config move_group.launch.py rviz:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('dual_panda', package_name='dualarm_moveit_config')
        .robot_description_semantic(file_path='config/dual_panda.srdf')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )
    sim_time = {'use_sim_time': True}

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), sim_time],
    )

    # Add the "MotionPlanning" display in RViz to plan interactively.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Also start RViz.'),
        move_group,
        rviz,
    ])
