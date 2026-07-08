# Inicia o Gazebo Harmonic (gz-sim) carregando o mundo do pacote.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('pioneer3at')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(pkg_share, 'worlds', 'pioneer_world.sdf')

    # Permite ao gz-sim resolver caminhos package://pioneer3at/...
    # (a pasta-pai de pkg_share contém o diretório 'pioneer3at').
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg_share))

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Caminho do arquivo .sdf do mundo a carregar.')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true = roda só o servidor (sem GUI).')

    # gz_args: "<mundo> -r" (-r = começar rodando). "-s" quando headless.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [world, ' -r -v 4'],
            #'gz_version': '8',
        }.items(),
    )

    return LaunchDescription([
        resource_path,
        declare_world,
        declare_headless,
        gz_sim,
    ])
