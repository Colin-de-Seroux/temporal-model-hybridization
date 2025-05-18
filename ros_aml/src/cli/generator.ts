import { type Model, type Node } from '../language/generated/ast.js';
import { CompositeGeneratorNode, toString } from 'langium/generate';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { camelCaseToSnakeCase, extractDestinationAndName } from './cli-util.js';
import {
    generateDockerfile,
    generateDockerComposePart,
} from './generator_files/docker_file_generator.js';
import { generateEntrypoint } from './generator_files/entrypoint_generator.js';
import { generateJsonGraphModel } from './generator_files/graph_generator.js';
import { generatePackageXml } from './generator_files/xml_generator.js';
import {
    generateSetupCfg,
    generateSetupPy,
} from './generator_files/setup_generator.js';
import { generateTimerExecutionPy } from './generator_files/timer_execution_generator.js';
import { generateLaunchFile } from './generator_files/launch_generation.js';

export function generateRosScript(
    model: Model,
    filePath: string,
    destination: string | undefined
): string {
    const data = extractDestinationAndName(filePath, destination);
    const pkgName = camelCaseToSnakeCase(data.name);
    const rootPath = path.join(data.destination, pkgName);

    const srcPath = path.join(rootPath, pkgName);
    fs.mkdirSync(srcPath, { recursive: true });
    fs.mkdirSync(path.join(rootPath, 'resource'), { recursive: true });

    fs.writeFileSync(path.join(srcPath, '__init__.py'), '');

    model.nodes.forEach((node) => {
        const fileNode = compileNode(pkgName, node);
        const nodeFilePath = path.join(
            srcPath,
            `${camelCaseToSnakeCase(node.name)}.py`
        );
        fs.writeFileSync(nodeFilePath, toString(fileNode));
    });
    fs.writeFileSync(
        path.join(rootPath, 'graph.json'),
        generateJsonGraphModel(model)
    );
    fs.writeFileSync(
        path.join(srcPath, 'timer_execution.py'),
        generateTimerExecutionPy()
    );
    fs.writeFileSync(
        path.join(rootPath, 'setup.py'),
        generateSetupPy(pkgName, model)
    );
    fs.writeFileSync(
        path.join(rootPath, 'setup.cfg'),
        generateSetupCfg(pkgName)
    );
    fs.writeFileSync(
        path.join(rootPath, 'package.xml'),
        generatePackageXml(pkgName)
    );
    fs.writeFileSync(path.join(rootPath, 'resource', pkgName), '');
    fs.writeFileSync(
        path.join(rootPath, 'Dockerfile'),
        generateDockerfile(pkgName)
    );
    fs.writeFileSync(
        path.join(rootPath, 'docker-compose.yml'),
        generateDockerComposePart(pkgName)
    );
    fs.writeFileSync(
        path.join(rootPath, 'entrypoint.sh'),
        generateEntrypoint(pkgName, model.logger.level)
    );
    fs.mkdirSync(path.join(rootPath, 'launch'), { recursive: true });
    fs.writeFileSync(
        path.join(rootPath, 'launch', `${pkgName}_launch.py`),
        generateLaunchFile(pkgName, model.nodes, model.logger.level)
    );

    return rootPath;
}

function compileNode(pkgName: string, node: Node): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();

    const constructorBody = new CompositeGeneratorNode();
    const callbackFunctions = new CompositeGeneratorNode();
    const serviceActionFunctions = new CompositeGeneratorNode();

    nodeBlock.append(`import rclpy`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from rclpy.node import Node`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from rclpy.executors import ExternalShutdownException`);
    nodeBlock.appendNewLine();
    nodeBlock.append(
        `from std_msgs.msg import String, Int32, Float32, Bool, Header`
    );
    nodeBlock.appendNewLine();
    nodeBlock.append(`import time`);
    nodeBlock.appendNewLine();
    nodeBlock.append(
        `from ${pkgName}.timer_execution import measure_execution_time`
    );

    nodeBlock.appendNewLine();
    nodeBlock.appendNewLine();
    nodeBlock.appendNewLine();

    nodeBlock.append(`class ${node.name}Node(Node):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def __init__(self):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        super().__init__('${node.name}')`);
    nodeBlock.appendNewLine();

    node.init?.steps.forEach((step) => {
        if (step.$type === 'Logger') {
            constructorBody.append(
                `        self.get_logger().${step.level}('${step.msg}')`
            );
            constructorBody.appendNewLine();
        } else if (step.$type === 'Publisher') {
            constructorBody.append(
                `        self.${step.name} = self.create_publisher(${step.msgType}, '${step.topic}', ${step.qos})`
            );
            constructorBody.appendNewLine();
        } else if (step.$type === 'Subscriber') {
            constructorBody.append(
                `        self.${step.name} = self.create_subscription(${step.msgType}, '${step.topic}', self.${step.callbackFunction}, ${step.qos})`
            );
            constructorBody.appendNewLine();
        } else if (step.$type === 'Timer') {
            constructorBody.append(
                `        self.${step.name} = self.create_timer(${step.period}, self.${step.callbackFunction})`
            );
            constructorBody.appendNewLine();
        }
    });

    nodeBlock.append(constructorBody);

    node.callbackFunctions?.forEach((callback) => {
        callbackFunctions.appendNewLine();
        callbackFunctions.append(`    @measure_execution_time`);
        callbackFunctions.appendNewLine();
        callbackFunctions.append(
            `    def ${callback.name}(self${callback.type === 'Subscriber' ? ', msg' : ''}):`
        );
        callbackFunctions.appendNewLine();

        callback.steps?.steps.forEach((step) => {
            if (step.$type === 'Logger') {
                callbackFunctions.append(
                    `        self.get_logger().${step.level}(${step.msg})`
                );
                callbackFunctions.appendNewLine();
            } else if (step.$type === 'Publish') {
                callbackFunctions.append(
                    `        self.${step.publisherName}.publish(${step.msg})`
                );
                callbackFunctions.appendNewLine();
            } else if (step.$type === 'Service' || step.$type === 'Action') {
                // TODO: Implement Service and Action
                callbackFunctions.append(`        self.${step.name}()`);
                callbackFunctions.appendNewLine();

                serviceActionFunctions.appendNewLine();
                serviceActionFunctions.append(`    @measure_execution_time`);
                serviceActionFunctions.appendNewLine();
                serviceActionFunctions.append(`    def ${step.name}(self):`);
                serviceActionFunctions.appendNewLine();
                serviceActionFunctions.append(
                    `        self.get_logger().info('${step.name} called')`
                );
                serviceActionFunctions.appendNewLine();
                serviceActionFunctions.append(
                    `        time.sleep(${Number(step.estimatedExecutionTime) / 1000})`
                );
                serviceActionFunctions.appendNewLine();
            }
        });
    });

    nodeBlock.append(callbackFunctions);
    nodeBlock.append(serviceActionFunctions);

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
