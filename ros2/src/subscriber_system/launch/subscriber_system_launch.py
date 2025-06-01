from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='subscriber_system',
            executable='subscriber',
            name='SubscriberNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        LogInfo(
            condition=None,
            msg="Launch file executed successfully!"
        )
    ])