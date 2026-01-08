from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch fleet_adapter_sony.

    Usage example:
        ros2 launch fleet_adapter_sony fleet_adapter.launch.py \
            config_file:=/root/rmf_ws/src/fleet_adapter_sony/config.yaml \
            nav_graph:=/root/rmf_ws/src/your_nav_graph.yaml \
            use_sim_time:=true
    """

    config_file = LaunchConfiguration('config_file')
    nav_graph = LaunchConfiguration('nav_graph')
    server_uri = LaunchConfiguration('server_uri')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            description='Path to fleet_adapter_sony config.yaml',
        ),
        DeclareLaunchArgument(
            'nav_graph',
            description='Path to RMF nav_graph.yaml for this fleet',
        ),
        DeclareLaunchArgument(
            'server_uri',
            default_value='',
            description='(Optional) URI of api server to transmit state and task info',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use Gazebo/Sim time (true/false)',
        ),
        Node(
            package='fleet_adapter_sony',
            executable='fleet_adapter',
            name='fleet_adapter_sony',
            output='screen',
            arguments=[
                '-c', config_file,
                '-n', nav_graph,
                '-s', server_uri,
                '--use_sim_time', use_sim_time,
            ],
        ),
    ])
