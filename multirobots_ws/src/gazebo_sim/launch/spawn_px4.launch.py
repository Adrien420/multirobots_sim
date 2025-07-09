#!/usr/bin/env python3

import os
import re


from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def replace_model_uri(match):
    model_name = match.group(1)
    rest = match.group(2)

    return f'package://px4/models/{model_name}{rest}'


def patch_nested_model(robot_desc:str, model_name:str):
    model_file = "/home/multirobots/multirobots_ws/src/PX4-Autopilot/Tools/simulation/gz/models/" + model_name + "/model.sdf"

    with open(model_file, 'r') as infp:
        tmp_model_file = infp.read()

    tmp_model_file = re.sub(r'model://([^/<\s]+)(/[^<\s]*)?', replace_model_uri, tmp_model_file)
    
    for i in range(4):
        tmp_model_file = tmp_model_file.replace(
            f'<joint name="rotor_{i}_joint" type="revolute">',
            f'<joint name="rotor_{i}_joint" type="fixed">'
        )

    with open('/tmp/' + model_name, 'w') as outfp:
        outfp.write(tmp_model_file)
        
    robot_desc = robot_desc.replace(
        'model://' + model_name,
        '/tmp/' + model_name
    )

    return robot_desc

def launch_setup(context):
    drone_id = int(LaunchConfiguration('drone_id').perform(context))
    x_pose = int(LaunchConfiguration('x_pose').perform(context))

    sdf_file = "/home/multirobots/multirobots_ws/src/PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf"

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Adapt the sdf file to the limitations of the sdformat_urdf package
    robot_desc = patch_nested_model(robot_desc, "x500_base")
    robot_desc = patch_nested_model(robot_desc, "lidar_2d_v2")
    robot_desc = patch_nested_model(robot_desc, "mono_cam")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="px1",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}]
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            #'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'world/forest/model/x500_{drone_id}/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/world/forest/model/x500_{drone_id}/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/world/forest/model/x500_{drone_id}/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image',

            '--ros-args', '--log-level', 'info'
        ],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    spawn_px4_cmd = ExecuteProcess(
        cmd=[
            '/home/multirobots/multirobots_ws/src/PX4-Autopilot/build/px4_sitl_default/bin/px4',
            '-i', str(drone_id)
        ],
        env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'gz_x500',
            'PX4_GZ_MODEL_POSE': [
                str(x_pose),
                TextSubstitution(text=',0,1,0,0,0')
            ],
            'PX4_GZ_WORLD': 'forest',
            'PATH': os.environ.get('PATH', '') + ':/home/multirobots/multirobots_ws/src/PX4-Autopilot/build/px4_sitl_default/bin',
        },
        output='screen',
    )

    spawn_px4_cmds = []

    spawn_px4_cmds.append(robot_state_publisher_node)
    spawn_px4_cmds.append(gz_bridge)
    spawn_px4_cmds.append(spawn_px4_cmd)

    return spawn_px4_cmds

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('x_pose', default_value='0'),
        OpaqueFunction(function=launch_setup)
    ])