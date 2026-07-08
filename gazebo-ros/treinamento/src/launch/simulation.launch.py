# Bringup completo: Gazebo Harmonic + robô + ponte (+ RViz2 opcional).
#
#   ros2 launch pioneer3at simulation.launch.py
#   ros2 launch pioneer3at simulation.launch.py rviz:=true
#   ros2 launch pioneer3at simulation.launch.py world:=/caminho/outro.sdf
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('pioneer3at')
    launch_dir = os.path.join(pkg_share, 'launch')

    default_world = os.path.join(pkg_share, 'worlds', 'mezanino_graco.sdf')
    rviz_config = os.path.join(pkg_share, 'config', 'pioneer3at.rviz')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false',
                                         description='true = abre o RViz2.')

    # Resolve package:// para o gz-sim
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=os.path.dirname(pkg_share))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={'world': world}.items())

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_robot.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bridge.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        # use_sim_time como booleano explícito
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(rviz),
        output='screen')

    # Inicia o RViz só DEPOIS que o Gazebo e a ponte estão no ar (e o /clock
    # já está fluindo). Isso evita o "jump back in time": sem o atraso, o RViz
    # arranca no relógio do sistema e dá um salto para trás quando o primeiro
    # /clock (tempo de simulação) chega, limpando o buffer de TF (o robô pisca).
    delayed_rviz = TimerAction(period=5.0, actions=[rviz_node])

    return LaunchDescription([
        resource_path,
        declare_world,
        declare_use_sim_time,
        declare_rviz,
        gazebo,
        spawn,
        bridge,
        delayed_rviz,
    ])
