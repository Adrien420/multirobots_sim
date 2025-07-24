import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

def launch_setup(context):
    nb_summits = int(LaunchConfiguration('nb_summits').perform(context))

    spawn_summits_cmds = []

    # Include launch file spawning summit xl with its id, nb_drones times 
    for i in range(1, nb_summits+1):
        spawn_summits_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'spawn_summit.launch.py')
            ),
            launch_arguments={
                'summit_id':str(i),
                'x_pose':str(i-1)
            }.items()
        )
        
        if i==1:
            spawn_summits_cmds.append(spawn_summits_cmd)
        else: # Wait for the previous summit xl to be instantiated, so that its controllers can be spawned correctly
            wait_summit_ready_cmd = Node(
                package='gazebo_sim',
                executable='wait_topic_creation',
                parameters=[{"use_sim_time": True, "robot_type":"summit", "summit_id":i-1}],
                output='screen',
            )
            spawn_summits_cmds.append(wait_summit_ready_cmd)
            spawn_summits_cmds.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=wait_summit_ready_cmd,
                        on_exit=[spawn_summits_cmd]
                    ),
                )
            )
        
    return spawn_summits_cmds

def generate_launch_description():
    
    ld = LaunchDescription([
        DeclareLaunchArgument('nb_summits', default_value='2'),
        OpaqueFunction(function=launch_setup),
    ])

    return ld