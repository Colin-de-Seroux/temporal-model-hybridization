from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test',
            executable='talker',
            name='TalkerNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        Node(
            package='test',
            executable='listeningistener',
            name='ListeningistenerNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        Node(
            package='test',
            executable='service_handler',
            name='ServiceHandlerNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        Node(
            package='test',
            executable='action_executor',
            name='ActionExecutorNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        LogInfo(
            condition=None,
            msg="Launch file executed successfully!"
        )
    ])