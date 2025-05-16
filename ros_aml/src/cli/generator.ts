import {
    ActivationPattern,
    Behavior,
    Model,
    Node,
    Parameter,
    State,
    Timer,
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
import { generateJsonModel } from './generator_files/json_generator.js';
import { generatePackageXml } from './generator_files/xml_generator.js';
import {
    generateSetupCfg,
    generateSetupPy,
} from './generator_files/setup_generator.js';
import { generateTimerExecutionPy } from './generator_files/timer_execution_generator.js';
import { generateLaunchFile } from './generator_files/launch_generation.js';
import { generateValueCode, getBehaviorMethodName } from './utils/utils.js';

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
        generateJsonModel(model)
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
    const initBlock = new CompositeGeneratorNode();
    const methodsBlock = new CompositeGeneratorNode();
    const node_name_lower_case = node.name.toLowerCase()

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
    nodeBlock.appendNewLine();



    nodeBlock.append(`class ${node.name}Node(Node):`);
    nodeBlock.appendNewLine();

    nodeBlock.append(`    def __init__(self):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        super().__init__('${node_name_lower_case}')`);
    nodeBlock.appendNewLine();

    if (node.activation) {
            initBlock.append(compileActivation(node.activation));
        }

    for (const param of node.params) {
        initBlock.append(compileParameter(param));
    }

    for (const state of node.states) {
        initBlock.append(compileState(state))
    }

    for (const timer of node.timers) {
        initBlock.append(compileTimer(timer));
    }

    initBlock.appendNewLine();

    for (const behavior of node.behaviors) {
        const [initCode, methodCode] = compileBehavior(behavior);
        initBlock.append(initCode);
        methodsBlock.append(methodCode);
    }
    nodeBlock.append(initBlock);
    nodeBlock.appendNewLine();
    nodeBlock.append(methodsBlock)
    nodeBlock.append(compileMain(node));

    return nodeBlock;
}


function generateActionCode(action: any): [CompositeGeneratorNode, CompositeGeneratorNode]  {
    switch (action.$type) {
        case 'SendMessage':
            return generateSendMessageCode(action);
        case 'LogMessage':
            return generateLogMessageCode(action);
        case 'CallService':
            return generateCallServiceCode(action);
        case 'SetParam':
            return generateSetParamCode(action);
        case 'GetParam':
            return generateGetParamCode(action);
        case 'UpdateState':
            return generateUpdateStateCode(action);
        default:
            const init = new CompositeGeneratorNode();
            const exec = new CompositeGeneratorNode();
            exec.append(`        # Action non gérée`);
            return [init, exec];
    }
}

function generateSendMessageCode(action: any): [CompositeGeneratorNode, CompositeGeneratorNode]  {
    const topic = action.topic;
    const msg = action.message;
    
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();
    //const qos = action.qos ? (action.qos === 'reliable' ? 'reliable' : 'best_effort') : 'best_effort';

    init.append(`        self.publisher_${topic} = self.create_publisher(String, '${topic}', 10)`);
    init.appendNewLine();

    exec.append(`        self.publisher_${topic}.publish(String(data='${msg}'))`);
    exec.appendNewLine();

    return [init, exec];
}

function generateLogMessageCode(action: any): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const level = action.level || 'info';
    const msg = action.message.replace(/"/g, '\\"');
    exec.append(`        self.get_logger().${level}("${msg}")`);
    exec.appendNewLine();

    return [init, exec];
}

function generateCallServiceCode(action: any): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const srv = action.service;
    const req = action.request.replace(/"/g, '\\"');

    init.append(`        self.client_${srv} = self.create_client(ServiceType, '${srv}')`);
    init.appendNewLine();

    const callServiceLines = [
        `        while not self.client_${srv}.wait_for_service(timeout_sec=1.0):`,
        `            self.get_logger().warn('Service not available, waiting...')`,
        `        req = ServiceType.Request()`,
        `        req.data = ${req}`,
        `        future = self.client_${srv}.call_async(req)`,
        `        rclpy.spin_until_future_complete(self, future)`,
        `        if future.result() is not None:`,
        `            self.get_logger().info('Service call succeeded')`,
        `        else:`,
        `            self.get_logger().error('Service call failed')`
    ];
    for (const line of callServiceLines) {
        exec.append(line);
        exec.appendNewLine();
    }

    return [init, exec];
}


function generateSetParamCode(action: any): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const param = action.param;
    const valCode = generateValueCode(action.value);

    exec.append(`        self.set_parameters([rclpy.parameter.Parameter('${param}', value=${valCode})])`);
    exec.appendNewLine();

    return [init, exec];
}

function generateGetParamCode(action: any): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const param = action.param;
    exec.append(`        val = self.get_parameter('${param}').value`);
    exec.appendNewLine();
    exec.append(`        self.get_logger().info(f"Param ${param} = {val}")`);
    exec.appendNewLine();

    return [init, exec];
}

function generateUpdateStateCode(action: any): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const state = action.state;
    const valCode = generateValueCode(action.value);
    exec.append(`        self.${state} = ${valCode}`);
    exec.appendNewLine();

    return [init, exec];
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

function compileActivation(activation: ActivationPattern) : CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    if (activation.timer) {
        nodeBlock.append(compilePeridodicActivation(activation.timer));
            
    } else if (activation.source) {
        nodeBlock.append(compileEventDrivenActivation(activation.source));
        
    }
    return nodeBlock;
}

function compilePeridodicActivation(period: string): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    const periodSec = typeof period === 'number' ? (period / 1000).toFixed(3) : '1.000';
    nodeBlock.append(
        `        # Activation périodique avec période ${period ?? 'undefined'} ms`
    );
    nodeBlock.appendNewLine();
    nodeBlock.append(
        `        self.activation_timer = self.create_timer(${periodSec}, self.${period}_callback)`
    );
    nodeBlock.appendNewLine();
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

function compileBehavior(behavior: Behavior): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const initCode = new CompositeGeneratorNode();
    const methodCode = new CompositeGeneratorNode();
    const methodName = getBehaviorMethodName(behavior);

    methodCode.appendNewLine();
    methodCode.append(`    @measure_execution_time()`);
    methodCode.appendNewLine();
    methodCode.append(`    def ${methodName}(self):`);
    methodCode.appendNewLine();

    const [actionInit, actionExec] = generateActionCode(behavior.action);
    initCode.append(actionInit);
    methodCode.append(actionExec);
    methodCode.appendNewLine();

    if (behavior.trigger.timer) {
        compileTimerTrigger(behavior, methodCode,methodName);   
    }
    if (behavior.trigger.topic) {
        compileTopicTrigger(behavior, methodCode, methodName);
    }
    if (behavior.trigger.service) {
        compileServiceTrigger(behavior, methodCode, methodName);
    }
    if (behavior.trigger.action) {
        compileActionTrigger(behavior, methodCode, methodName);
    }
    if (behavior.trigger.param) {
        compileParamTrigger(behavior, methodCode, methodName);
    }
    if (behavior.trigger.state) {
        compileStateTrigger(behavior, methodCode, methodName);
    }
    
    return [initCode, methodCode];
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

function compileTimerTrigger(behavior: Behavior, nodeBlock: CompositeGeneratorNode,methodName: string): void {
    const timerName = behavior.trigger.timer;
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def ${timerName}_callback(self):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        self.${methodName}()`);
    nodeBlock.appendNewLine();
}

function compileTopicTrigger(behavior: Behavior, nodeBlock: CompositeGeneratorNode,methodName: string): void {
    const topicName = behavior.trigger.topic;
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def ${topicName}_callback(self, msg):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        self.${methodName}()`);
    nodeBlock.appendNewLine();
}

function compileServiceTrigger(behavior: Behavior, nodeBlock: CompositeGeneratorNode,methodName: string): void {
    const serviceName = behavior.trigger.service;
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def ${serviceName}_callback(self, request, response):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        self.${methodName}()`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        response.success = True`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        response.message = "Traitement lancé"`);   
    nodeBlock.appendNewLine();    
    nodeBlock.append(`        return response`);
    nodeBlock.appendNewLine();
}

function compileActionTrigger(behavior: Behavior, nodeBlock: CompositeGeneratorNode,methodName: string): void {
    const actionName = behavior.trigger.action;
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def ${actionName}_callback(self, goal_handle):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        self.${methodName}()`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        goal_handle.succeed()`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        result = ... # TODO: définir le type et la réponse`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        return result`);
    nodeBlock.appendNewLine();
}

function compileParamTrigger(behavior: Behavior, nodeBlock: CompositeGeneratorNode,methodName: string): void {
    const paramName = behavior.trigger.param;
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def on_parameter_event(self, event):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        for changed_param in event.changed_parameters:`); 
    nodeBlock.appendNewLine();
    nodeBlock.append(`            if changed_param.name == '${paramName}':`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`                self.${methodName}()`);
    nodeBlock.appendNewLine();
}

function compileStateTrigger(behavior: Behavior, nodeBlock: CompositeGeneratorNode,methodName: string): void {
    const stateName = behavior.trigger.state;
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def on_state_changed(self):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        if self.${stateName} == True:`); 
    nodeBlock.appendNewLine();
    nodeBlock.append(`            self.${methodName}()`);
    nodeBlock.appendNewLine();
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