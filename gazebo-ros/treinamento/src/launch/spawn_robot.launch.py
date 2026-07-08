# Publica a descrição do robô (robot_state_publisher) e faz o spawn no gz-sim.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('pioneer3at')
    xacro_file = os.path.join(pkg_share, 'urdf', 'pioneer3at.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('x')
    spawn_y = LaunchConfiguration('y')
    spawn_z = LaunchConfiguration('z')
    spawn_yaw = LaunchConfiguration('yaw')

    # Processa o xacro em tempo de launch -> string URDF
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Usar o relógio da simulação (/clock).')
    declare_x = DeclareLaunchArgument('x', default_value='9')
    declare_y = DeclareLaunchArgument('y', default_value='3.6')
    declare_z = DeclareLaunchArgument('z', default_value='0.15')
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='1.5708',
        description='Orientação inicial (yaw) do robô em RADIANOS.')

    # Publica TF dos links fixos + tópico /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    # Faz o spawn no Gazebo lendo o tópico /robot_description
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'pioneer3at',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        robot_state_publisher,
        spawn_entity,
    ])
