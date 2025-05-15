import {
    Action,
    ActivationPattern,
    Behavior,
    Model,
    Node,
    Parameter,
    State,
    Timer,
    Value,
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
        generateEntrypoint(pkgName, model.logger?.level ?? 'info')
    );
    fs.mkdirSync(path.join(rootPath, 'launch'), { recursive: true });
    fs.writeFileSync(
        path.join(rootPath, 'launch', `${pkgName}_launch.py`),
        generateLaunchFile(pkgName, model.nodes, model.logger?.level ?? 'info')
    );

    return rootPath;
}

function compileNode(pkgName: string, node: Node): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    //const constructorBody = new CompositeGeneratorNode();
    //const methods = new CompositeGeneratorNode();

    nodeBlock.append(`import rclpy`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from rclpy.node import Node`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from rclpy.executors import ExternalShutdownException`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from std_msgs.msg import String, Int32, Float32, Bool, Header`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from ${pkgName}.timer_execution import measure_execution_time`);
    nodeBlock.appendNewLine();

    nodeBlock.append(`class ${node.name}Node(Node):`);
    nodeBlock.appendNewLine();

    nodeBlock.append(`    def __init__(self):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        super().__init__('${node.name}')`);
    nodeBlock.appendNewLine();

    if (node.activation) {
            nodeBlock.append(compileActivation(node.activation));
        }

    for (const param of node.params) {
        nodeBlock.append(compileParameter(param));
    }

    for (const state of node.states) {
        nodeBlock.append(compileState(state))
    }

    for (const timer of node.timers) {
        nodeBlock.append(compileTimer(timer));
    }

    nodeBlock.appendNewLine();

    for (const behavior of node.behaviors) {
        nodeBlock.append(compileBehavior(behavior));       
    }

    nodeBlock.append(compileMain(node));

    return nodeBlock;
}

function getBehaviorMethodName(behavior: Behavior): string {
    if (behavior.trigger.timer) return `on_timerElapsed_${behavior.trigger.timer}`;
    if (behavior.trigger.topic) return `on_messageReceived_${behavior.trigger.topic}`;
    if (behavior.trigger.service) return `on_serviceRequest_${behavior.trigger.service}`;
    if (behavior.trigger.action) return `on_actionGoalReceived_${behavior.trigger.action}`;
    if (behavior.trigger.param) return `on_paramChanged_${behavior.trigger.param}`;
    if (behavior.trigger.state) return `on_stateChanged_${behavior.trigger.state}`;
    return 'on_unknown_trigger';
}

function generateActionCode(action: Action): string {
    switch (action.$type) {
        case 'SendMessage':
            return `        # Envoi d'un message\n        self.get_logger().info("sendMessage not implemented yet")`;
        case 'LogMessage':
            return `        self.get_logger().${action.level}('${action.message}')`;
        case 'CallService':
            return `        # Appel de service ${action.service}\n        self.get_logger().info("callService not implemented yet")`;
        case 'SetParam':
            return `        self.set_parameters([rclpy.parameter.Parameter('${action.param}', value=${generateValueCode(action.value)})])`;
        case 'GetParam':
            return `        val = self.get_parameter('${action.param}').value\n        self.get_logger().info(f"Param ${action.param} = {val}")`;
        case 'UpdateState':
            return `        self.${action.state} = ${generateValueCode(action.value)}`;
        default:
            return `        # Action non gérée`;
    }
}

function generateValueCode(value: Value | undefined): string {
    if (!value) return 'None'; 

    if (value.intValue !== undefined) return value.intValue.toString();
    if (value.doubleValue !== undefined) return value.doubleValue.toString();
    if (value.stringValue !== undefined) return `'${value.stringValue}'`; 
    if (value.boolValue !== undefined) return value.boolValue === 'true' ? 'True' : 'False';

    return 'None'; 
}



function compileParameter(param: Parameter): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    const defaultVal = generateValueCode(param.defaultValue )?? 'None';
    nodeBlock.append(`        self.declare_parameter('${param.name}', ${defaultVal})`);
    nodeBlock.appendNewLine();

    return nodeBlock;
}

function compileState(state: State): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    const initialVal = generateValueCode(state.initialValue) ?? 'None';
    nodeBlock.append(`        self.${state.name} = ${initialVal}`);
    nodeBlock.appendNewLine();

    return nodeBlock;
}

function compileTimer(timer: Timer): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    const periodSec = (Number(timer.period) / 1000).toFixed(3);
    nodeBlock.append(
        `        self.${timer.name}_timer = self.create_timer(${periodSec}, self.${timer.name}_callback)`
    );
    nodeBlock.appendNewLine();

    return nodeBlock;
}

function compilePeridodicActivation(period: number): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    const periodSec = typeof period === 'number' ? (period / 1000).toFixed(3) : '1.000';
    nodeBlock.append(
        `        # Activation périodique avec période ${period ?? 'undefined'} ms`
    );
    nodeBlock.appendNewLine();
    nodeBlock.append(
        `        self.activation_timer = self.create_timer(${periodSec}, self.activation_callback)`
    );
    nodeBlock.appendNewLine();
    return nodeBlock;
}

function compileActivation(activation: ActivationPattern) : CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    if (activation.period) {
        nodeBlock.append(compilePeridodicActivation(Number(activation.period)));
            
    } else if (activation.source) {
        nodeBlock.append(compileEventDrivenActivation(activation.source));
        
    }
    return nodeBlock;
}

function compileEventDrivenActivation(source: string): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    nodeBlock.append(
        `        # Activation event-driven sur la source '${source}'`
    );
    nodeBlock.appendNewLine();
    nodeBlock.append(
        `        self.create_subscription(String, '${source}', self.activation_callback, 10)`
    );
    nodeBlock.appendNewLine();
    return nodeBlock;
}

function compileBehavior(behavior: Behavior): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    const methodName = getBehaviorMethodName(behavior);
    nodeBlock.appendNewLine();
    nodeBlock.append(`    @measure_execution_time()`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def ${methodName}(self):`);
    nodeBlock.appendNewLine();

    nodeBlock.append(generateActionCode(behavior.action));
    nodeBlock.appendNewLine();

    nodeBlock.append(`    `);

    if (behavior.trigger.timer) {
        const timerName = behavior.trigger.timer;
        nodeBlock.appendNewLine();
        nodeBlock.append(`    def ${timerName}_callback(self):`);
        nodeBlock.appendNewLine();
        nodeBlock.append(`        self.${methodName}()`);
        nodeBlock.appendNewLine();
    }

    if (behavior.trigger.topic) {
        const topicName = behavior.trigger.topic;
        nodeBlock.appendNewLine();
        nodeBlock.append(`    def ${topicName}_callback(self, msg):`);
        nodeBlock.appendNewLine();
        nodeBlock.append(`        self.${methodName}()`);
        nodeBlock.appendNewLine();
    }
    return nodeBlock;
}

function compileMain(node: Node): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
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
/*
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
*/