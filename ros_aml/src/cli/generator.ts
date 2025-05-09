import {  Publisher, Subscriber, type Model, type Node } from '../language/generated/ast.js';
import { CompositeGeneratorNode,  toString } from 'langium/generate';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { extractDestinationAndName } from './cli-util.js';

export function generateRosScript(model: Model, filePath: string, destination: string | undefined): string {
    const data = extractDestinationAndName(filePath, destination);
    const pkgName = data.name.toLowerCase();
    const rootPath = path.join(data.destination, pkgName);

    const srcPath = path.join(rootPath, pkgName);
    fs.mkdirSync(srcPath, { recursive: true });
    fs.mkdirSync(path.join(rootPath, 'resource'), { recursive: true });

    fs.writeFileSync(path.join(srcPath, '__init__.py'), '');

    model.nodes.forEach((node) => {
        const fileNode = compileNode(pkgName,node);
        const nodeFilePath = path.join(srcPath, `${node.name}.py`);
        fs.writeFileSync(nodeFilePath, toString(fileNode));
    });
    fs.writeFileSync(path.join(srcPath, 'timer_execution.py'), generateTimerExecutionPy());

    fs.writeFileSync(path.join(rootPath, 'setup.py'), generateSetupPy(pkgName, model));
    fs.writeFileSync(path.join(rootPath, 'setup.cfg'), generateSetupCfg(pkgName));
    fs.writeFileSync(path.join(rootPath, 'package.xml'), generatePackageXml(pkgName));
    fs.writeFileSync(path.join(rootPath, 'resource', pkgName), '');
    fs.writeFileSync(path.join(rootPath, 'Dockerfile'), generateDockerfile(pkgName));
    fs.writeFileSync(path.join(rootPath, 'entrypoint.sh'), generateEntrypoint(pkgName));

    return rootPath;
}

function generateSetupPy(pkgName: string, model: Model): string {
    return `
from setuptools import find_packages,setup

package_name = '${pkgName}'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + pkgName]),
        ('share/'+pkgName, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto-generated',
    maintainer_email='noreply@example.com',
    description='Auto-generated ROS 2 package',
    license="Apache 2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            ${model.nodes.map(n => `'${n.name.toLowerCase()} = ${pkgName}.${n.name}:main'`).join(',\n            ')}
        ],
    },
)
`.trim();
}

function generateSetupCfg(pkgName: string): string {
    return `
[develop]
script_dir=$base/lib/${pkgName}
[install]
install_scripts=$base/lib/${pkgName}
`.trim();
}

function generatePackageXml(pkgName: string): string {
    return `
<package format="3">
  <name>${pkgName}</name>
  <version>0.0.1</version>
  <description>Auto-generated ROS 2 package</description>
  <maintainer email="noreply@example.com">Auto Generator</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>python3</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
`.trim();
}

function generateDockerfile(pkgName: string): string {
    return `
FROM ros:jazzy

RUN apt-get update && apt-get install -y \\
    python3-colcon-common-extensions \\
    python3-pip \\
    build-essential \\
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Create logs dir
RUN mkdir -p /ros2_ws/.ros/log

# Copy ${pkgName} package
COPY . /ros2_ws/src/${pkgName}

# Install dependencies
RUN . /opt/ros/jazzy/setup.sh && \\
    colcon build --merge-install

# New entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Source the workspace, install dependencies and run the node
CMD . /opt/ros/jazzy/setup.sh && \\
    . /ros2_ws/install/setup.sh && \\
    ros2 run ${pkgName} ${pkgName}
`.trim();
}

function generateEntrypoint(pkgName: string): string {
    return `
#!/bin/bash

LOG_DIR="/ros2_ws/.ros/log"
BACKUP_DIR="/ros_logs_backup"

function save_logs {
    echo "Saving ROS2 logs to disk..."
    mkdir -p "$BACKUP_DIR"
    cp -r "$LOG_DIR"/* "$BACKUP_DIR"/
    echo "Logs saved to $BACKUP_DIR."
}

# Set trap to call save_logs on exit
trap save_logs EXIT

# Run ROS2 node
echo "Starting ROS2 node..."
source /opt/ros/jazzy/setup.sh
source /ros2_ws/install/setup.sh
ros2 run ${pkgName} ${pkgName} > /dev/null 2>&1
`.trim();
}

function generateTimerExecutionPy(): string {
    return `
import functools
from rclpy.clock import Clock
from rclpy.logging import get_logger

def measure_execution_time(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        logger = get_logger("timer_execution")
        try:
            clock = Clock()
            start = clock.now()
            result = func(*args, **kwargs)
            end = clock.now()
            duration = (end - start).nanoseconds / 1e6
            logger.info(f"Time execution of function {func.__name__} : {duration:.2f} ms")
            return result
        except Exception as e:
            logger.error(f"Error in {func.__name__}: {e}")
            raise
    return wrapper
`.trim();
}

function compileNode(pkgName:string ,node: Node): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();

    nodeBlock.append(`import rclpy`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from rclpy.node import Node`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from rclpy.executors import ExternalShutdownException`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from std_msgs.msg import *`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from ${pkgName}.timer_execution import measure_execution_time`);
    nodeBlock.appendNewLine();
    nodeBlock.appendNewLine();


    nodeBlock.append(`class ${node.name}Node(Node):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def __init__(self):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        super().__init__('${node.name}')`);
    nodeBlock.appendNewLine();



    node.publishers?.forEach(pub => {
        nodeBlock.append(compilePublisher(pub));
        nodeBlock.appendNewLine();
    });

    node.subscribers?.forEach(sub => {
        nodeBlock.append(compileSubscriber(sub));
        nodeBlock.appendNewLine();
    });
    nodeBlock.appendNewLine();
    nodeBlock.append(`
def main(args=None):
    rclpy.init(args=args)
    node = ${node.name}Node()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
`);
    return nodeBlock;

}


    

function compilePublisher(pub: Publisher): CompositeGeneratorNode {
    const node = new CompositeGeneratorNode();
    const type = pub.msgType.split('.').at(-1) ?? pub.msgType;
    node.append(`        self.publisher_${pub.topicName} = self.create_publisher(${type}, '${pub.topicName}', 10)`);
    return node;
}

function compileSubscriber(sub: Subscriber): CompositeGeneratorNode {
    const node = new CompositeGeneratorNode();
    const type = sub.msgType.split('.').at(-1) ?? sub.msgType;
    node.append(`        self.subscription_${sub.topicName} = self.create_subscription(${type}, '${sub.topicName}', self.listener_callback_${sub.topicName}, 10)`);
    return node;
}


/*function compileService(service: Service, fileNode: CompositeGeneratorNode): void {
    fileNode.append(`import rclpy`);

}*/




