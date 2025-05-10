import {  Publisher, Subscriber, type Model, type Node } from '../language/generated/ast.js';
import { CompositeGeneratorNode,  toString } from 'langium/generate';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { extractDestinationAndName } from './cli-util.js';
import { generateDockerfile } from './generator_files/docker_file_generator.js';
import { generateEntrypoint } from './generator_files/entrypoint_generator.js';
import { generatePackageXml } from './generator_files/xml_generator.js';
import { generateSetupCfg, generateSetupPy } from './generator_files/setup_generator.js';
import { generateTimerExecutionPy } from './generator_files/timer_execution_generator.js';

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




