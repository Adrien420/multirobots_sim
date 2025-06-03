#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    
    drone_id = LaunchConfiguration('drone_id')
    x_pose = LaunchConfiguration('x_pose')
    
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('x_pose', default_value='0'),
        ExecuteProcess(
            cmd=[
                '/home/multirobots/multirobots_ws/src/PX4-Autopilot/build/px4_sitl_default/bin/px4',
                '-i', drone_id
            ],
            env={
                'PX4_GZ_STANDALONE': '1',
                'PX4_SYS_AUTOSTART': '4001',
                'PX4_SIM_MODEL': 'gz_x500',
                'PX4_GZ_MODEL_POSE': [
                    x_pose,
                    TextSubstitution(text=',0,1,0,0,0')
                ],
                'PX4_GZ_WORLD': 'forest',
                'PATH': os.environ.get('PATH', '') + ':/home/multirobots/multirobots_ws/src/PX4-Autopilot/build/px4_sitl_default/bin',
            },
            output='screen',
        ),
])