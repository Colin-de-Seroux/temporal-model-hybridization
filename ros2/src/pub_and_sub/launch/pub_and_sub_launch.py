from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pub_and_sub',
            executable='pub_sub',
            name='PubSubNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        LogInfo(
            condition=None,
            msg="Launch file executed successfully!"
        )
    ])