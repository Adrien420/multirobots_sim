#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):

    x = LaunchConfiguration('x_pose')
    y = LaunchConfiguration('y_pose')
    z = LaunchConfiguration('z_pose')
    summit_id = int(LaunchConfiguration('summit_id').perform(context))
    
    spawn_summits_cmds = []
    
    #namespace = f'summit_xl_{summit_id}'
    namespace = ""
    
     # Path to the xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare("summit_xl_description"),
        "robots",
        "summit_xl_std.urdf.xacro"
    ])

    # Robot's description (URDF generated from the .xacro thanks to the command xacro)
    #robot_description_config = ParameterValue(Command(['xacro', ' ', xacro_file, ' robot_namespace:=', namespace]), value_type=str)
    robot_description_config = ParameterValue(Command(['xacro', ' ', xacro_file]), value_type=str)

    #robot_state_publisher_node
    spawn_summits_cmds.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[{"robot_description": robot_description_config}]
        )
    )
    
    #gz_bridge
    spawn_summits_cmds.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            arguments=[
                'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                'world/forest/model/summit_xl/link/summit_xl_base_footprint/sensor/summit_xl_front_laser_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                #'--ros-args', '-p', 'expand_gz_topic_names:=true', '-r', f'__ns:=/summit_xl_{summit_id}',
                '--ros-args',
                '--log-level', 'info'
            ],
            # parameters=[os.path.join(get_package_share_directory("gazebo_sim"), "config", "ros_gz_bridge.yaml")], Doesn't work
            output='screen'
        )
    )
    
    #spawn_summit
    spawn_summits_cmds.append(
        Node(
            package="ros_gz_sim",
            executable="create",
            namespace=namespace,
            arguments=[
                '-name', f'{namespace}',
                '-topic', 'robot_description',
                '-x', x,
                '-y', y,
                '-z', z
            ],
            output='screen'
        )
    )
    
    #diff_drive_spawner
    spawn_summits_cmds.append(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["robotnik_base_controller"],
        )
    )

    #joint_broad_spawner
    spawn_summits_cmds.append(
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace,
            arguments=["joint_state_broadcaster"],
        )
    )
    
    return spawn_summits_cmds

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='1.0'),
        DeclareLaunchArgument('z_pose', default_value='1.0'),
        DeclareLaunchArgument('summit_id', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])