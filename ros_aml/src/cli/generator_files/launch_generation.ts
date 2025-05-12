export function generateLaunchFile(pkgName: string, nodeName: string): string {
    return `
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='${pkgName}',
            executable='${nodeName}',
            name='${nodeName}',
            output='screen'
        )
    ])
`.trim();
}
