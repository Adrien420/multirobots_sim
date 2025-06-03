#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Récupération des paramètres de pose
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    # Chemin vers le fichier xacro ou urdf
    xacro_file = PathJoinSubstitution([
        FindPackageShare("summit_xl_description"),
        "robots",
        "summit_xl_std.urdf"
    ])

    # Description robot (URDF à partir d’un .urdf ou d’un .xacro)
    robot_description_config = ParameterValue(Command(['xacro', ' ', xacro_file]), value_type=str)

    # Node robot_state_publisher avec robot_description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_config}]
    )

    # Spawner via ros_gz_sim
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            '-name', 'summit_xl',
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='1.0'),
        DeclareLaunchArgument('z', default_value='1.0'),
        robot_state_publisher_node,
        spawn_entity
    ])