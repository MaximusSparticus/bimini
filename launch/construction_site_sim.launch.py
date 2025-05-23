import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('bimini')
    bimbot_share = get_package_share_directory('bimbot')

    # Path to world file
    world_file = 'generated_worlds/construction_site_from_ifc.sdf'

    # Launch Gazebo with the generated world
    #gz_sim = ExecuteProcess(
    #    cmd=['gz', 'sim', '-r', world_file],
    #    output='screen'
    #)

    # Include robot spawning launch file
    spawn_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bimbot_share, 'launch', 'spawn_robots.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'spawn_builderbot': 'true',
            'spawn_inspectorbot': 'true'
        }.items()
    )

    return LaunchDescription([
        #gz_sim,
        spawn_robots
    ])
