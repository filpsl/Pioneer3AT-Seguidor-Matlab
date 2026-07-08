# Sobe a ponte ros_gz_bridge (parameter_bridge) usando o YAML de configuração.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pioneer3at')
    default_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_config = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='YAML de mapeamento de tópicos da ponte.')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        declare_config,
        declare_use_sim_time,
        bridge,
    ])
