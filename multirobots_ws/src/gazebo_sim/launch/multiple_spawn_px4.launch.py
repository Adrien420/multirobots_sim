import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context):
    nb_drones = int(LaunchConfiguration('nb_drones').perform(context))

    spawn_drones_cmds = []

    for i in range(1, nb_drones+1):
        spawn_drones_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'spawn_px4.launch.py')
                ),
                launch_arguments={
                    'drone_id':str(i),
                    'x_pose':str(i-1)
                }.items()
            )
        )
        
    return spawn_drones_cmds

def generate_launch_description():
    
    ld = LaunchDescription([
        DeclareLaunchArgument('nb_drones', default_value='4'),
        OpaqueFunction(function=launch_setup)
    ])

    return ld