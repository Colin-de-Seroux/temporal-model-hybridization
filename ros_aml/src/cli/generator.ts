import {
    Action,
    Callback,
    Publisher,
    Service,
    Subscriber,
    type Model,
    type Node,
} from '../language/generated/ast.js';
import { CompositeGeneratorNode, toString } from 'langium/generate';
import * as fs from 'node:fs';
import * as path from 'node:path';
import { camelCaseToSnakeCase, extractDestinationAndName } from './cli-util.js';
import {
    generateDockerfile,
    generateDockerComposePart,
} from './generator_files/docker_file_generator.js';
import { generateEntrypoint } from './generator_files/entrypoint_generator.js';
import { generatePackageXml } from './generator_files/xml_generator.js';
import {
    generateSetupCfg,
    generateSetupPy,
} from './generator_files/setup_generator.js';
import { generateTimerExecutionPy } from './generator_files/timer_execution_generator.js';
import { generateLaunchFile } from './generator_files/launch_generation.js';
import { resolveMessageType } from './utils/utils.js';

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
    const methods = new CompositeGeneratorNode();
    const types = new Set<string>();

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

    node.logs?.forEach((log) => {
        constructorBody.append(compileLogger(log.level, log.msg));
        constructorBody.appendNewLine();
    });

    node.publishers?.forEach((pub) => {
        constructorBody.append(compilePublisher(pub, types));
        constructorBody.appendNewLine();
    });

    node.subscribers?.forEach((sub) => {
        const [initCode, methodCode] = compileSubscriber(sub, types);
        constructorBody.append(initCode);
        constructorBody.appendNewLine();
        methods.append(methodCode);
        methods.appendNewLine();
    });

    node.services?.forEach((service) => {
        const [initCode, methodCode] = compileService(service, types);
        constructorBody.append(initCode);
        constructorBody.appendNewLine();
        methods.append(methodCode);
        methods.appendNewLine();
    });

    node.actions?.forEach((action) => {
        const [initCode, methodCode] = compileAction(action, types);
        constructorBody.append(initCode);
        constructorBody.appendNewLine();
        methods.append(methodCode);
        methods.appendNewLine();
    });

    node.callbacks?.forEach((callback) => {
        const [initCode, methodCode] = compileCallback(callback);
        constructorBody.append(initCode);
        constructorBody.appendNewLine();
        methods.append(methodCode);
        methods.appendNewLine();
    });


    nodeBlock.append(constructorBody);
    nodeBlock.append(methods);

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

function compileLogger(level: string, msg: string): CompositeGeneratorNode {
    const node = new CompositeGeneratorNode();
    node.append(`        self.get_logger().${level}('${msg}')`);
    return node;
}

function compilePublisher(
    pub: Publisher,
    types: Set<string>
): CompositeGeneratorNode {
    const node = new CompositeGeneratorNode();
    const type = resolveMessageType(pub.msgType, pub.msg);
    types.add(type);
    node.append(
        `        self.publisher_${pub.topicName} = self.create_publisher(${type}, '${pub.topicName}', 10)`
    );
    return node;
}

function compileSubscriber(
    sub: Subscriber,
    types: Set<string>
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const method = new CompositeGeneratorNode();
    const type = resolveMessageType(sub.msgType, sub.msg);
    types.add(type);

    init.append(
        `        self.subscription_${sub.topicName} = self.create_subscription(`
    );
    init.append(
        `${type}, '${sub.topicName}', self.listener_callback_${sub.topicName}, 10)`
    );
    init.appendNewLine();

    method.append(`    def listener_callback_${sub.topicName}(self, msg):`);
    method.appendNewLine();
    method.append(`        self.get_logger().info('${sub.msg}')`);
    method.appendNewLine();

    return [init, method];
}

function compileService(
    service: Service,
    types: Set<string>
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const method = new CompositeGeneratorNode();
    const type = service.srvType?.split('.').at(-1) ?? 'DefaultServiceType';
    types.add(type);

    init.append(
        `        self.service_${service.serviceName} = self.create_service(`
    );
    init.append(
        `${type}, '${service.serviceName}', self.handle_${service.serviceName})`
    );
    init.appendNewLine();

    method.append(
        `    def handle_${service.serviceName}(self, request, response):`
    );
    method.appendNewLine();
    method.append(`        # TODO: Implement service logic`);
    method.appendNewLine();
    method.append(`        return response`);
    method.appendNewLine();

    return [init, method];
}

function compileAction(
    action: Action,
    types: Set<string>
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const method = new CompositeGeneratorNode();
    const type = action.actionType?.split('.').at(-1) ?? 'DefaultActionType';
    types.add(type);
    init.append(`        # Action server for ${action.actionName}`);
    init.appendNewLine();
    init.append(
        `        self.action_server_${action.actionName} = rclpy.action.ActionServer(`
    );
    init.appendNewLine();
    init.append(
        `            self, ${type}, '${action.actionName}', self.execute_${action.actionName})`
    );
    init.appendNewLine();

    method.append(`    def execute_${action.actionName}(self, goal_handle):`);
    method.appendNewLine();
    method.append(
        `        self.get_logger().info('Executing goal: ${action.goal}')`
    );
    method.appendNewLine();
    method.append(`        # TODO: Add feedback publishing`);
    method.appendNewLine();
    method.append(`        goal_handle.succeed()`);
    method.appendNewLine();
    method.append(`        result = ${type}.Result()`);
    method.appendNewLine();
    method.append(`        result.result = "${action.result}"`);
    method.appendNewLine();
    method.append(`        return result`);
    method.appendNewLine();

    return [init, method];
}

function compileCallback(
    callback: Callback
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const method = new CompositeGeneratorNode();

    const callbackName = callback.name;
    const kind = callback.kind;
    const target = callback.target ?? callbackName;
    const execTime = callback.expectedExecTime;

    if (kind === 'timer') {
        init.append(`        self.timer_${callbackName} = self.create_timer(`);
        init.append(`${Number(execTime) / 1000}, self.callback_${callbackName})`);
        init.appendNewLine();

        method.append(`    @measure_execution_time()`);
        method.appendNewLine();
        method.append(`    def callback_${callbackName}(self):`);
        method.appendNewLine();
        method.append(`        self.get_logger().info('Timer callback ${callbackName} triggered')`);
        method.appendNewLine();

    } else if (kind === 'subscriber') {
        init.append(`        self.subscription_${target} = self.create_subscription(`);
        init.append(`String, '${target}', self.callback_${callbackName}, 10)`);
        init.appendNewLine();

        method.append(`    @measure_execution_time()`);
        method.appendNewLine();
        method.append(`    def callback_${callbackName}(self, msg):`);
        method.appendNewLine();
        method.append(`        self.get_logger().info('Subscriber callback ${callbackName} received: ' + str(msg.data))`);
        method.appendNewLine();

    } else if (kind === 'service') {
        init.append(`        self.service_${target} = self.create_service(`);
        init.append(`Empty, '${target}', self.callback_${callbackName})`);
        init.appendNewLine();

        method.append(`    @measure_execution_time()`);
        method.appendNewLine();
        method.append(`    def callback_${callbackName}(self, request, response):`);
        method.appendNewLine();
        method.append(`        self.get_logger().info('Service callback ${callbackName} triggered')`);
        method.appendNewLine();
        method.append(`        return response`);
        method.appendNewLine();

    } else if (kind === 'action') {
        init.append(`        self.action_server_${target} = rclpy.action.ActionServer(self, Empty, '${target}', self.callback_${callbackName})`);
        init.appendNewLine();

        method.append(`    @measure_execution_time()`);
        method.appendNewLine();
        method.append(`    def callback_${callbackName}(self, goal_handle):`);
        method.appendNewLine();
        method.append(`        self.get_logger().info('Action callback ${callbackName} executing')`);
        method.appendNewLine();
        method.append(`        goal_handle.succeed()`);
        method.appendNewLine();
        method.append(`        result = Empty.Result()`); 
        method.appendNewLine();
        method.append(`        return result`);
        method.appendNewLine();
    }

    return [init, method];
}
