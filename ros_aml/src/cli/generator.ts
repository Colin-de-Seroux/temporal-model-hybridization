import {
    ActionGoalReceived,
    ActivationPattern,
    Behavior,
    CallService,
    isActionGoalReceived,
    isCallService,
    isGetParam,
    isLogMessage,
    isMessageReceived,
    isParamChanged,
    isSendActionGoal,
    isSendMessage,
    isServiceRequest,
    isSetParam,
    isStateChanged,
    isTimerElapsed,
    isUpdateState,
    MessageReceived,
    Model,
    Node,
    ParamChanged,
    Parameter,
    SendActionGoal,
    SendMessage,
    ServiceRequest,
    State,
    StateChanged,
    Timer,
    TimerElapsed,
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
    const node_name_lower_case = node.name.toLowerCase();

    nodeBlock.append(`import rclpy`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from rclpy.node import Node`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`import time`);
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
    nodeBlock.append(`import json`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`from datetime import datetime`);
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
        initBlock.append(compileState(state));
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
    nodeBlock.append(methodsBlock);
    nodeBlock.append(compileMain(node));

    return nodeBlock;
}

function generateActionCode(
    action: any
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    if (isSendMessage(action)) {
        return generateSendMessageCode(action);
    } else if (isLogMessage(action)) {
        return generateLogMessageCode(action);
    } else if (isCallService(action)) {
        return generateCallServiceCode(action);
    } else if (isSetParam(action)) {
        return generateSetParamCode(action);
    } else if (isGetParam(action)) {
        return generateGetParamCode(action);
    } else if (isUpdateState(action)) {
        return generateUpdateStateCode(action);
    } else if (isSendActionGoal(action)) {
        return generateSendActionGoalCode(action);
    } else {
        const init = new CompositeGeneratorNode();
        const exec = new CompositeGeneratorNode();
        exec.append(`        #  Action non gérée`);
        exec.appendNewLine();
        return [init, exec];
    }
}

function generateSendMessageCode(
    action: SendMessage
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const topic = action.topic;
    const to_send = action.message;

    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();
    //const qos = action.qos ? (action.qos === 'reliable' ? 'reliable' : 'best_effort') : 'best_effort';

    init.append(
        `        self.publisher_${topic} = self.create_publisher(String, '${topic}', 10)`
    );
    init.appendNewLine();

    const level = 'info';
    exec.append(`        start_time = time.time()`);
    exec.appendNewLine();
    exec.append(`        self.get_logger().${level}("Send at: " + str(start_time))`);
    exec.appendNewLine();

    exec.append(
        `        self.publisher_${topic}.publish(String(data='${to_send}'))`
    );
    exec.appendNewLine();

    return [init, exec];
}

function generateLogMessageCode(
    action: any
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const level = action.level || 'info';
    const msg = action.message.replace(/"/g, '\\"');
    exec.append(`        self.get_logger().${level}("${msg}")`);
    exec.appendNewLine();

    return [init, exec];
}

function generateCallServiceCode(
    action: CallService
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    /*const srv = action.service;
    const req = action.request.replace(/"/g, '\\"');

    init.append(
        `        self.client_${srv} = self.create_client(ServiceType, '${srv}')`
    );
    init.appendNewLine();

    const callServiceLines = [
        `        while not self.client_${srv}.wait_for_service(timeout_sec=1.0):`,
        `            self.get_logger().warn('Service not available, waiting...')`,
        `        req = ServiceType.Request()`,
        `        req.data = json.dump(${req})`,
        `        future = self.client_${srv}.call_async(req)`,
        `        rclpy.spin_until_future_complete(self, future)`,
        `        if future.result() is not None:`,
        `            self.get_logger().info('Service call succeeded')`,
        `        else:`,
        `            self.get_logger().error('Service call failed')`,
    ];
    for (const line of callServiceLines) {
        exec.append(line);
        exec.appendNewLine();
    }*/

    const execTime = action.expectedTime; 
    exec.append(
        `        time.sleep(${Number(execTime)/1000}) `
    );

    return [init, exec];
}

function generateSetParamCode(
    action: any
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const param = action.param;
    const valCode = generateValueCode(action.value);

    exec.append(
        `        self.set_parameters([rclpy.parameter.Parameter('${param}', value=${valCode})])`
    );
    exec.appendNewLine();

    return [init, exec];
}

function generateGetParamCode(
    action: any
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const param = action.param;
    exec.append(`        val = self.get_parameter('${param}').value`);
    exec.appendNewLine();
    exec.append(`        self.get_logger().info(f"Param ${param} = {val}")`);
    exec.appendNewLine();

    return [init, exec];
}

function generateUpdateStateCode(
    action: any
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    const state = action.state;
    const valCode = generateValueCode(action.value);
    exec.append(`        self.${state} = ${valCode}`);
    exec.appendNewLine();

    return [init, exec];
}

function generateSendActionGoalCode(
    action: SendActionGoal
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const init = new CompositeGeneratorNode();
    const exec = new CompositeGeneratorNode();

    /*const actionName = action.action;
    const goal = action.goal.replace(/"/g, '\\"');*/
    /*init.append(
        `        self.action_client_${actionName} = ActionClient(self, ${actionName}, '${actionName}')`
    );
    init.appendNewLine();

    exec.append(`        goal_msg = '${goal}'`);
    exec.appendNewLine();
    exec.append(
        `        self.action_client_${actionName}.send_goal_async(goal_msg)`
    );
    exec.appendNewLine();*/
    const execTime = action.expectedTime; 
    exec.append(
        `        time.sleep(${Number(execTime)/1000}) `
    );

    return [init, exec];
}

function compileParameter(param: Parameter): CompositeGeneratorNode {
    const nodeBlock = new CompositeGeneratorNode();
    const defaultVal = generateValueCode(param.defaultValue) ?? 'None';
    nodeBlock.append(
        `        self.declare_parameter('${param.name}', ${defaultVal})`
    );
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

function compileActivation(
    activation: ActivationPattern
): CompositeGeneratorNode {
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
    const periodSec =
        typeof period === 'number' ? (period / 1000).toFixed(3) : '1.000';
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

function compileBehavior(
    behavior: Behavior
): [CompositeGeneratorNode, CompositeGeneratorNode] {
    const initCode = new CompositeGeneratorNode();
    const methodCode = new CompositeGeneratorNode();
    const methodName = getBehaviorMethodName(behavior);
    const triger = behavior.trigger;

    methodCode.appendNewLine();
    methodCode.append(`    @measure_execution_time`);
    methodCode.appendNewLine();
    methodCode.append(`    def ${methodName}(self):`);
    methodCode.appendNewLine();

    for (const action of behavior.action) {
        const [actionInit, actionExec] = generateActionCode(action);
        initCode.append(actionInit);
        methodCode.append(actionExec);
        methodCode.appendNewLine();
    }

    if (isTimerElapsed(triger)) {
        compileTimerTrigger(triger, methodCode, methodName);
    }
    if (isMessageReceived(triger)) {
        compileTopicTrigger(triger, methodCode, initCode, methodName);
    }
    if (isServiceRequest(triger)) {
        compileServiceTrigger(triger, methodCode, methodName);
    }
    if (isActionGoalReceived(triger)) {
        compileActionTrigger(triger, methodCode, initCode, methodName);
    }
    if (isParamChanged(triger)) {
        compileParamTrigger(triger, methodCode, methodName);
    }
    if (isStateChanged(triger)) {
        compileStateTrigger(triger, methodCode, methodName);
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

function compileTimerTrigger(
    triger: TimerElapsed,
    nodeBlock: CompositeGeneratorNode,
    methodName: string
): void {
    const timerName = triger.timer;
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def ${timerName}_callback(self):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        self.${methodName}()`);
    nodeBlock.appendNewLine();
}

function compileTopicTrigger(
    triger: MessageReceived,
    nodeBlock: CompositeGeneratorNode,
    initCode: CompositeGeneratorNode,
    methodName: string
): void {
    const topicName = triger.topic;
    const messageType = 'String';
    initCode.appendNewLine();
    initCode.append(
        `        self.subscription_${topicName} = self.create_subscription(${messageType}, '${topicName}', self.${topicName}_callback, 10)`
    );
    initCode.appendNewLine();

    nodeBlock.appendNewLine();
    nodeBlock.append(`    def ${topicName}_callback(self, msg):`);
    nodeBlock.appendNewLine();
    const level = 'info';
    nodeBlock.append(`        finish_time = time.time()`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        self.get_logger().${level}("Received at: " + str(finish_time))`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        self.${methodName}()`);
    nodeBlock.appendNewLine();
}

function compileServiceTrigger(
    triger: ServiceRequest,
    nodeBlock: CompositeGeneratorNode,
    methodName: string
): void {
    const serviceName = triger.service;
    nodeBlock.appendNewLine();
    nodeBlock.append(
        `    def ${serviceName}_callback(self, request, response):`
    );
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

function compileActionTrigger(
    triger: ActionGoalReceived,
    nodeBlock: CompositeGeneratorNode,
    initBlock: CompositeGeneratorNode,
    methodName: string
): void {
    const actionName = triger.action;
    initBlock.appendNewLine();
    initBlock.append(`        self._${actionName}_server = ActionServer(`);
    initBlock.appendNewLine();
    initBlock.append(`            self,`);
    initBlock.appendNewLine();
    initBlock.append(
        `            YOUR_ACTION_TYPE,  # TODO: Remplacer par le bon type`
    );
    initBlock.appendNewLine();
    initBlock.append(`            '${actionName}',`);
    initBlock.appendNewLine();
    initBlock.append(`            self.${actionName}_callback`);
    initBlock.appendNewLine();
    initBlock.append(`        )`);
    initBlock.appendNewLine();

    nodeBlock.appendNewLine();
    nodeBlock.append(`    def ${actionName}_callback(self, goal_handle):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        goal= goal_handle.request`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        self.${methodName}()`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        goal_handle.succeed()`);
    nodeBlock.appendNewLine();
    nodeBlock.append(
        `        result = ... # TODO: définir le type et la réponse`
    );
    nodeBlock.appendNewLine();
    nodeBlock.append(`        result.success = True`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        return result`);
    nodeBlock.appendNewLine();
}

function compileParamTrigger(
    triger: ParamChanged,
    nodeBlock: CompositeGeneratorNode,
    methodName: string
): void {
    const paramName = triger.param;
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

function compileStateTrigger(
    triger: StateChanged,
    nodeBlock: CompositeGeneratorNode,
    methodName: string
): void {
    const stateName = triger.state;
    nodeBlock.appendNewLine();
    nodeBlock.append(`    def on_state_changed(self):`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`        if self.${stateName} == True:`);
    nodeBlock.appendNewLine();
    nodeBlock.append(`            self.${methodName}()`);
    nodeBlock.appendNewLine();
}
