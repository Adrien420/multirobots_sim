from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnShutdown

import os, datetime, subprocess
from ament_index_python import get_package_share_directory, get_package_prefix
# Warning : get_package_share_directory gives the following path : /.../share/pkg_name, and not /.../share

def on_shutdown(event, context):
    """Force QGroundControl to close on shutdown, without the need of a confirmation in the app"""
    subprocess.run(["pkill", "-9", "QGroundControl"])
    return

def launch_setup(context):
    
    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    use_rosbag = LaunchConfiguration('rosbag')
    nb_summits = int(LaunchConfiguration('nb_summits').perform(context))
    nb_rangers = int(LaunchConfiguration('nb_rangers').perform(context))
    nb_drones = int(LaunchConfiguration('nb_drones').perform(context))

    # Path to save rosbags
    now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    rosbag_save_dir = f'/home/multirobots/multirobots_ws/src/gazebo_sim/rosbag/record_{now}'
    
    # Commands

    # Launch gazebo with the forest world
    gazebo = ExecuteProcess(
        cmd=[
            'gz sim ',
            PathJoinSubstitution([
                FindPackageShare('gazebo_sim'),
                'worlds',
                'forest.sdf '
            ]),
            '-r', # Allow to start the simulation as soon as Gazebo is launched
            '2>&1 | grep -v "not defined in SDF"' # Remove some warnings caused by the plugin provided by the sdformat_urdf package
        ],
        shell=True
    )
    
    # Launch Rviz with a configuration file set for one robot of each type
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('gazebo_sim'), 'rviz', 'summit_xl_1.rviz')],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression([use_rviz, ' and "', LaunchConfiguration('nb_drones'), '" == "0"']))
    )
    
    # Bridge the clock topic for sim time
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args', '--log-level', 'info'
        ],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    # Topics of each robot simulated 
    topics_summits = []

    for i in range(1, nb_summits+1):
        topics_summits.append(f'summit_xl_{i}/scan')
        topics_summits.append(f'summit_xl_{i}/imu_data')
        topics_summits.append(f'summit_xl_{i}/navsat_data')
        topics_summits.append(f'summit_xl_{i}/color/image_raw')
        topics_summits.append(f'summit_xl_{i}/color/camera_info')
        topics_summits.append(f'summit_xl_{i}/depth/image_raw/image')
        topics_summits.append(f'summit_xl_{i}/depth/image_raw/depth_image')
        topics_summits.append(f'summit_xl_{i}/depth/image_raw/points')
        topics_summits.append(f'summit_xl_{i}/depth/image_raw/camera_info')
        topics_summits.append(f'summit_xl_{i}/robot_description')

    topics_rangers = []

    for i in range(1, nb_rangers+1):
        topics_rangers.append(f'ranger_mini_{i}/scan')
        topics_rangers.append(f'ranger_mini_{i}/imu_data')
        topics_rangers.append(f'ranger_mini_{i}/navsat_data')
        topics_rangers.append(f'ranger_mini_{i}/color/image_raw')
        topics_rangers.append(f'ranger_mini_{i}/color/camera_info')
        topics_rangers.append(f'ranger_mini_{i}/depth/image_raw/image')
        topics_rangers.append(f'ranger_mini_{i}/depth/image_raw/depth_image')
        topics_rangers.append(f'ranger_mini_{i}/depth/image_raw/points')
        topics_rangers.append(f'ranger_mini_{i}/depth/image_raw/camera_info')
        topics_rangers.append(f'ranger_mini_{i}/robot_description')

    topics_drones = []

    for i in range(1, nb_drones+1):
        topics_drones.append(f'world/forest/model/x500_{i}/link/link/sensor/lidar_2d_v2/scan')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/camera_link/sensor/imager/camera_info')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/camera_link/sensor/imager/image')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/depth_camera_link/sensor/IMX214/camera_info')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/depth_camera_link/sensor/IMX214/image')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/depth_camera_link/sensor/StereoOV7251/camera_info')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/depth_camera_link/sensor/StereoOV7251/depth_image')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/depth_camera_link/sensor/StereoOV7251/depth_image/points')
        topics_drones.append(f'/px{i}/robot_description')

    # If use_rosbag is set to true, register every topic passed in argument
    rosbag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', rosbag_save_dir,
            '--use-sim-time',
            'clock',
            *topics_summits,
            *topics_drones,
            *topics_rangers,
            'tf', 'tf_static'
        ],
        condition=IfCondition(use_rosbag),
        output='screen'
    )

    tf_publisher = Node(
        package='gazebo_sim',
        executable='tf_publisher',
        output='screen',
        parameters=[{"use_sim_time": True, "nb_drones":nb_drones}],
    )
    
    # PX4
    microXRCEagent_cmd = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent udp4 -p 8888'
        ],
        cwd="/home/multirobots/multirobots_ws/src/Micro-XRCE-DDS-Agent/build",
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('nb_drones'), '" != "0"'])),
        shell=True
    )
    
    QGroundControl_cmd = ExecuteProcess(
        cmd=[
            './squashfs-root/AppRun > /dev/null 2>&1'
        ],
        cwd = '/home/multirobots/multirobots_ws/src/QGroundControl',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('nb_drones'), '" != "0"'])),
        shell=True
    )
    
    px4_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'multiple_spawn_px4.launch.py')
        ),
        launch_arguments={'nb_drones':str(nb_drones)}.items()
    )
    
    # Summit XL
    summit_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'multiple_spawn_summit.launch.py')
        ),
        launch_arguments={'nb_summits':str(nb_summits)}.items(),
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('nb_drones'), '" == "0"'])),
    )

    # Wait for every drones to be ready before spawning the Summit XL
    wait_px4_ready_cmd = Node(
        package='gazebo_sim',
        executable='wait_topic_creation',
        parameters=[{"use_sim_time": True, "robot_type":"drone"}],
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('nb_drones'), '" != "0"'])),
    )

    delayed_summit_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'multiple_spawn_summit.launch.py')
        ),
        launch_arguments={'nb_summits':str(nb_summits)}.items(),
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('nb_drones'), '" != "0"'])),
    )

    event_handler_summit_spawner_cmd = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_px4_ready_cmd,
            on_exit=[delayed_summit_spawner_cmd]
        ),
    )

    # Wait for every drones to be ready before launching Rviz
    delayed_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('gazebo_sim'), 'rviz', 'summit_xl_1.rviz')],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression([use_rviz, ' and "', LaunchConfiguration('nb_drones'), '" != "0"']))
    )

    event_handler_rviz_cmd = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_px4_ready_cmd,
            on_exit=[delayed_rviz_cmd]
        ),
    )

    # Ranger Mini

    # Select the robot to wait according to which robots are being simulated
    robot_to_wait = ""
    if nb_summits > 0:
        robot_to_wait = "summit"
    else:
        robot_to_wait = "drone"

    ranger_mini_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'multiple_spawn_ranger_mini.launch.py')
        ),
        launch_arguments={'nb_rangers':str(nb_rangers)}.items(),
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration(f'nb_{robot_to_wait}s'), '" == "0"'])),
    )

    # Wait for every selected robots to be ready before spawning the Ranger Mini
    wait_ready_cmd = Node(
        package='gazebo_sim',
        executable='wait_topic_creation',
        parameters=[{"use_sim_time": True, "robot_type":f"{robot_to_wait}", "summit_id":nb_summits}],
        output='screen',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration(f'nb_{robot_to_wait}s'), '" != "0"'])),
    )

    delayed_ranger_mini_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'multiple_spawn_ranger_mini.launch.py')
        ),
        launch_arguments={'nb_rangers':str(nb_rangers)}.items(),
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration(f'nb_{robot_to_wait}s'), '" != "0"'])),
    )

    event_handler_ranger_mini_spawner_cmd = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_ready_cmd,
            on_exit=[delayed_ranger_mini_spawner_cmd]
        ),
    )

    # Call the on_shutdown function when shutting down the launch file
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=on_shutdown
        )
    )

    simu_cmds = []

    # Simulation
    simu_cmds.append(gazebo)
    simu_cmds.append(rviz)
    simu_cmds.append(event_handler_rviz_cmd)
    simu_cmds.append(gz_bridge)
    simu_cmds.append(rosbag)
    simu_cmds.append(tf_publisher)

    # PX4
    simu_cmds.append(microXRCEagent_cmd)
    simu_cmds.append(QGroundControl_cmd)
    simu_cmds.append(shutdown_handler)
    simu_cmds.append(px4_spawner_cmd)

    # Summit
    simu_cmds.append(summit_spawner_cmd)
    simu_cmds.append(wait_px4_ready_cmd)
    simu_cmds.append(event_handler_summit_spawner_cmd)

    # Ranger Mini
    simu_cmds.append(ranger_mini_spawner_cmd)
    simu_cmds.append(wait_ready_cmd)
    simu_cmds.append(event_handler_ranger_mini_spawner_cmd)

    return simu_cmds

def generate_launch_description():

    # Paths to the resources
    PX4__model_path = "/home/multirobots/multirobots_ws/src/PX4-Autopilot/Tools/simulation/gz/models"
    summit_model_path = os.path.join(get_package_prefix("summit_xl_description"), 'share')
    summit_sensors_model_path = os.path.join(get_package_prefix("robotnik_sensors"), 'share')
    world_path = os.path.join(get_package_prefix("gazebo_sim"), 'share')
    resource_path =  world_path + ':' + PX4__model_path + ':' + summit_model_path + ':' + summit_sensors_model_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    server_config_path = "/home/multirobots/multirobots_ws/src/PX4-Autopilot/Tools/simulation/gz/server.config"
    
    PX4_plugin_path = "/home/multirobots/multirobots_ws/src/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins/"
    summit_plugin_path = "/home/multirobots/multirobots_ws/install/gz_ros2_control/lib"
    plugins_path = PX4_plugin_path + ':' + summit_plugin_path + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SERVER_CONFIG_PATH", server_config_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins_path),
        DeclareLaunchArgument('rviz', default_value='False'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument('nb_summits', default_value='1'),
        DeclareLaunchArgument('nb_rangers', default_value='0'),
        DeclareLaunchArgument('nb_drones', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])
