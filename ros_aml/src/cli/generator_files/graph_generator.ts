import {
    type CallbackFunction,
    type Model,
    type Node,
    type Step,
} from '../../language/generated/ast.js';

export function generateJsonGraphModel(model: Model): string {
    console.log(model);
    const graph: any = {};

    graph['name'] = model.systemName;
    graph['loggerLevel'] = model.logger.level;
    graph['nodes'] = [];

    model.nodes.forEach((node) => {
        graph['nodes'].push(createNode(node));
    });

    const graphString = JSON.stringify(graph, null, 4);

    return graphString;
}

function createNode(node: Node): any {
    const jsonNode: any = {};
    jsonNode['name'] = node.name;
    jsonNode['expectedExecTime'] = node.expectedExecTime;

    jsonNode['init'] = [];
    node.init?.steps.forEach((step) => {
        jsonNode['init'].push(createStep(step));
    });

    jsonNode['callbackFunctions'] = [];
    node.callbackFunctions?.forEach((callback) => {
        jsonNode['callbackFunctions'].push(createCallbackFunction(callback));
    });

    return jsonNode;
}

function createCallbackFunction(callback: CallbackFunction): any {
    const jsonCallback: any = {};
    jsonCallback['name'] = callback.name;
    jsonCallback['type'] = callback.type;

    jsonCallback['steps'] = [];
    callback.steps?.steps.forEach((step) => {
        jsonCallback['steps'].push(createStep(step));
    });

    return jsonCallback;
}

function createStep(step: Step): any {
    // TODO
    return {};
}
