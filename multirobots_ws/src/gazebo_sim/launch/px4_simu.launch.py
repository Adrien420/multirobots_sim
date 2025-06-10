from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os
from ament_index_python import get_package_share_directory, get_package_prefix
# Attention : get_package_share_directory renvoit le chemin /.../share/pkg_name et non /.../share

def generate_launch_description():
    
    # Paths
    PX4__model_path = "/home/multirobots/multirobots_ws/src/PX4-Autopilot/Tools/simulation/gz/models"
    summit_model_path = os.path.join(get_package_prefix("summit_xl_description"), 'share')
    summit_sensors_model_path = os.path.join(get_package_prefix("robotnik_sensors"), 'share')
    world_path = os.path.join(get_package_prefix("gazebo_sim"), 'share')
    resource_path =  world_path + ':' + PX4__model_path + ':' + summit_model_path + ':' + summit_sensors_model_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    server_config_path = "/home/multirobots/multirobots_ws/src/PX4-Autopilot/Tools/simulation/gz/server.config"
    
    PX4_plugin_path = "/home/multirobots/multirobots_ws/src/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins/"
    summit_plugin_path = "/home/multirobots/multirobots_ws/install/gz_ros2_control/lib"
    plugins_path = PX4_plugin_path + ':' + summit_plugin_path + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    
    # Commands
    gazebo = ExecuteProcess(
        cmd=[
            'gz sim ',
            PathJoinSubstitution([
                FindPackageShare('gazebo_sim'),
                'worlds',
                'forest.sdf '
            ]),
            '-r', # Permet de lancer directement la simulation
        ],
        shell=True
    )
    
    
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/forest/model/summit_xl/link/summit_xl_base_footprint/sensor/summit_xl_front_laser_sensor/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '--ros-args', '--log-level', 'debug'
        ],
        #parameters=[os.path.join(get_package_share_directory("gazebo_sim"), "config", "ros_gz_bridge.yaml")],
        output='screen'
    )
        
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotnik_base_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    
    microXRCEagent_cmd = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent udp4 -p 8888'
        ],
        cwd="/home/multirobots/multirobots_ws/src/Micro-XRCE-DDS-Agent/build",
        shell=True
    )
    
    QGroundControl_cmd = ExecuteProcess(
        cmd=[
            './squashfs-root/AppRun > /dev/null 2>&1'
        ],
        cwd = '/home/multirobots/multirobots_ws/src/QGroundControl.AppImage',
        shell=True
    )
    
    px4_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'multiple_spawn_px4.launch.py')
        ),
        launch_arguments={'nb_drones':'2'}.items()
    )
    
    summit_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'spawn_summit.launch.py')
        )
    )

    ld = LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SERVER_CONFIG_PATH", server_config_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins_path),
        gz_bridge,
        diff_drive_spawner,
        joint_broad_spawner
    ])
    
    ld.add_action(gazebo)
    #ld.add_action(microXRCEagent_cmd)
    #ld.add_action(QGroundControl_cmd)
    #ld.add_action(px4_spawner_cmd)
    ld.add_action(summit_spawner_cmd)
    return ld
