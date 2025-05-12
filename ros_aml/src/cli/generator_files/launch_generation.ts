export function generateLaunchFile(
    pkgName: string,
    fileName: string,
    nodeName: string,
    loggerLevel: string
): string {
    return `
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='${pkgName}',
            executable='${fileName}',
            name='${nodeName}',
            output='screen',
            arguments=['--ros-args', '--log-level', '${loggerLevel.toUpperCase()}'],
        ),
        LogInfo(
            condition=None,
            msg="Launch file executed successfully!"
        )
    ])
`.trim();
}
