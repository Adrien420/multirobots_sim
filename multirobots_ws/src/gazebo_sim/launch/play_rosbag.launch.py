import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

def launch_setup(context):
    rosbag_dirname = str(LaunchConfiguration('name').perform(context))

    rosbag_save_dir = f'/home/multirobots/multirobots_ws/src/gazebo_sim/rosbag/{rosbag_dirname}'
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('gazebo_sim'), 'rviz', 'summit_xl_1.rviz')],
        parameters=[{"use_sim_time": True}]
    )

    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', rosbag_save_dir,
            '--clock'
        ],
        output='screen'
    )

    play_summit_cmds = []

    play_summit_cmds.append(rosbag_play)
    play_summit_cmds.append(rviz)

    return play_summit_cmds

def generate_launch_description():
    

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value=''),
        OpaqueFunction(function=launch_setup)
    ])