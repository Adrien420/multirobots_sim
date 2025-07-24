import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

def launch_setup(context):
    nb_rangers = int(LaunchConfiguration('nb_rangers').perform(context))

    spawn_rangers_cmds = []

    # Include launch file spawning ranger mini with its id, nb_drones times 
    for i in range(1, nb_rangers+1):
        spawn_rangers_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ranger_mini'), 'launch', 'ranger_mini_v2_gazebo.launch.py')
            ),
            launch_arguments={
                'ranger_id':str(i),
                'x_pose':str(i-1),
                'y_pose':str(2)
            }.items()
        )
        
        if i==1:
            spawn_rangers_cmds.append(spawn_rangers_cmd)
        else: # Wait for the previous ranger mini to be instantiated, so that its controllers can be spawned correctly
            wait_ranger_ready_cmd = Node(
                package='gazebo_sim',
                executable='wait_topic_creation',
                parameters=[{"use_sim_time": True, "robot_type":"ranger", "ranger_id":i-1}],
                output='screen',
            )
            spawn_rangers_cmds.append(wait_ranger_ready_cmd)
            spawn_rangers_cmds.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=wait_ranger_ready_cmd,
                        on_exit=[spawn_rangers_cmd]
                    ),
                )
            )
        
    return spawn_rangers_cmds

def generate_launch_description():
    
    ld = LaunchDescription([
        DeclareLaunchArgument('nb_rangers', default_value='2'),
        OpaqueFunction(function=launch_setup),
    ])

    return ld