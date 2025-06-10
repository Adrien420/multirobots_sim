import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context):
    nb_summits = int(LaunchConfiguration('nb_summits').perform(context))

    spawn_summits_cmds = []

    for i in range(1, nb_summits+1):
        spawn_summits_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('gazebo_sim'), 'launch', 'spawn_summit.launch.py')
                ),
                launch_arguments={
                    'summit_id':str(i),
                    'x_pose':str(i-1)
                }.items()
            )
        )
        
    return spawn_summits_cmds

def generate_launch_description():
    
    ld = LaunchDescription([
        DeclareLaunchArgument('nb_summits', default_value='1'),
        OpaqueFunction(function=launch_setup),
    ])

    return ld