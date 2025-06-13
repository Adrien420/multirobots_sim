#!/usr/bin/env python3

import os, yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_yaml_with_namespace(context, summit_id):
    namespace = f"summit_xl_{summit_id}"
    original_yaml_path = os.path.join(
        FindPackageShare("summit_xl_description").perform(context),
        "config",
        "summit_control.yaml"
    )
    namespaced_yaml_path = f"/tmp/{namespace}_control.yaml"

    with open(original_yaml_path, 'r') as f:
        yaml_data = yaml.safe_load(f)

    namespaced_yaml = {}
    for key, value in yaml_data.items():
        namespaced_key = f"{namespace}/{key}"
        namespaced_yaml[namespaced_key] = value

    with open(namespaced_yaml_path, 'w') as f:
        yaml.dump(namespaced_yaml, f)

    return namespaced_yaml_path

def launch_setup(context):

    x = LaunchConfiguration('x_pose')
    y = LaunchConfiguration('y_pose')
    z = LaunchConfiguration('z_pose')
    summit_id = int(LaunchConfiguration('summit_id').perform(context))
    
    spawn_summits_cmds = []
    
    namespace = f'summit_xl_{summit_id}'
    
    yaml_namespace_path = generate_yaml_with_namespace(context, summit_id)
    
    print(yaml_namespace_path)
    
     # Path to the xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("summit_xl_description"),
        "robots",
        "summit_xl_std.urdf.xacro"
    ])

    # Robot's description (URDF generated from the .xacro thanks to the command xacro)
    robot_description_config = ParameterValue(Command(['xacro', ' ', xacro_file, ' robot_namespace:=', namespace]), value_type=str)
    #robot_description_config = ParameterValue(Command(['xacro', ' ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"robot_description": robot_description_config, "use_sim_time": True}]
    )
    
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'world/forest/model/summit_xl_{summit_id}/link/summit_xl_base_footprint/sensor/summit_xl_front_laser_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '--ros-args', '-p', 'expand_gz_topic_names:=true', '-r', f'__ns:=/summit_xl_{summit_id}',
            '--log-level', 'info'
        ],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )
    
    spawn_summit = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        arguments=[
            '-name', f'{namespace}',
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '--ros-args', '--log-level', 'info'
        ],
        output='screen'
    )
    
    robotnik_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["robotnik_base_controller"],
        parameters=[{"use_sim_time": True}],
    )

    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}],
    )
    
    spawn_summits_cmds.append(robot_state_publisher_node)
    spawn_summits_cmds.append(gz_bridge)
    spawn_summits_cmds.append(spawn_summit)
    spawn_summits_cmds.append(robotnik_controller_spawner)
    spawn_summits_cmds.append(joint_broadcaster_spawner)
    
    return spawn_summits_cmds

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='1.0'),
        DeclareLaunchArgument('z_pose', default_value='1.0'),
        DeclareLaunchArgument('summit_id', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])