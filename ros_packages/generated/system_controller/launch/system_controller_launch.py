from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='system_controller',
            executable='sensor',
            name='SensorNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        Node(
            package='system_controller',
            executable='processing',
            name='ProcessingNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        Node(
            package='system_controller',
            executable='actuator',
            name='ActuatorNode',
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO'],
        ),
        LogInfo(
            condition=None,
            msg="Launch file executed successfully!"
        )
    ])