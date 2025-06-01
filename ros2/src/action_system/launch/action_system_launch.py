from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='action_system',
            executable='action_server',
            name='ActionServerNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        Node(
            package='action_system',
            executable='action_client',
            name='ActionClientNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        LogInfo(
            condition=None,
            msg="Launch file executed successfully!"
        )
    ])