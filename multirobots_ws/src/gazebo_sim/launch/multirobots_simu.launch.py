from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

import os, datetime
from ament_index_python import get_package_share_directory, get_package_prefix
# Attention : get_package_share_directory renvoit le chemin /.../share/pkg_name et non /.../share

def launch_setup(context):
    
    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    use_rosbag = LaunchConfiguration('rosbag')
    nb_summits = int(LaunchConfiguration('nb_summits').perform(context))
    nb_drones = int(LaunchConfiguration('nb_drones').perform(context))

    now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    rosbag_save_dir = f'/home/multirobots/multirobots_ws/src/gazebo_sim/rosbag/record_{now}'
    
    # Commands
    gazebo = ExecuteProcess(
        cmd=[
            'gz sim ',
            PathJoinSubstitution([
                FindPackageShare('gazebo_sim'),
                'worlds',
                'forest2.sdf '
            ]),
            '-r', # Allow to start the simulation as soon as Gazebo is launched
            '2>&1 | grep -v "not defined in SDF"'
        ],
        shell=True
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('gazebo_sim'), 'rviz', 'summit_xl_1.rviz')],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz)
    )
    
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

    topics_summits = []

    for i in range(1, nb_summits+1):
        topics_summits.append(f'summit_xl_{i}/scan_{i}')
        topics_summits.append(f'summit_xl_{i}/imu_data_{i}')
        topics_summits.append(f'summit_xl_{i}/navsat_data_{i}')
        topics_summits.append(f'summit_xl_{i}/color/image_raw_{i}')
        topics_summits.append(f'summit_xl_{i}/color/camera_info')
        topics_summits.append(f'summit_xl_{i}/robot_description')

    topics_drones = []

    for i in range(1, nb_drones+1):
        topics_drones.append(f'world/forest/model/x500_{i}/link/link/sensor/lidar_2d_v2/scan')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/camera_link/sensor/imager/camera_info')
        topics_drones.append(f'/world/forest/model/x500_{i}/link/camera_link/sensor/imager/image')
        topics_drones.append(f'/px{i}/robot_description')

    rosbag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', rosbag_save_dir,
            '--use-sim-time',
            'clock',
            *topics_summits,
            *topics_drones,
            'tf', 'tf_static'
        ],
        condition=IfCondition(use_rosbag),
        output='screen'
    )

    static_tf_publisher = Node(
        package='gazebo_sim',
        executable='static_tf_publisher',
        output='screen',
        parameters=[{"use_sim_time": True}],
    )
    
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
        cwd = '/home/multirobots/multirobots_ws/src/QGroundControl.AppImage',
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('nb_drones'), '" != "0"'])),
        shell=True
    )
    
    px4_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'multiple_spawn_px4.launch.py')
        ),
        launch_arguments={'nb_drones':str(nb_drones)}.items()
    )
    
    summit_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'multiple_spawn_summit.launch.py')
        ),
        launch_arguments={'nb_summits':str(nb_summits)}.items(),
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('nb_drones'), '" == "0"'])),
    )

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

    simu_cmds = []

    # Simulation
    simu_cmds.append(gazebo)
    simu_cmds.append(rviz)
    simu_cmds.append(gz_bridge)
    simu_cmds.append(rosbag)
    simu_cmds.append(static_tf_publisher)

    # PX4
    simu_cmds.append(microXRCEagent_cmd)
    simu_cmds.append(QGroundControl_cmd)
    simu_cmds.append(px4_spawner_cmd)

    # Summit
    simu_cmds.append(summit_spawner_cmd)
    simu_cmds.append(wait_px4_ready_cmd)
    simu_cmds.append(event_handler_summit_spawner_cmd)

    return simu_cmds

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

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SERVER_CONFIG_PATH", server_config_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugins_path),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument('nb_summits', default_value='1'),
        DeclareLaunchArgument('nb_drones', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])
