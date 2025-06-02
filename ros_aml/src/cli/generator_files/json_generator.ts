import {
    Trigger,
    type Behavior,
    type Model,
    type Timer,
} from '../../language/generated/ast.js';

export function generateJsonModel(model: Model): string {
    const graph: any = {};

    graph['name'] = model.systemName;
    graph['nodes'] = [];

    model.nodes.forEach((node) => {
        const jsonNode: any = {};
        jsonNode['name'] = node.name;
        jsonNode['expectedExecTime'] = Number(node.expectedExecTime);
        jsonNode['timers'] = [];
        jsonNode['behaviors'] = [];

        for (const timer of node.timers) {
            jsonNode['timers'].push(generateTimer(timer));
        }

        for (const behavior of node.behaviors) {
            jsonNode['behaviors'].push(generateBehavior(behavior));
        }

        graph['nodes'].push(jsonNode);
    });

    const graphString = JSON.stringify(graph, null, 4);

    return graphString;
}

function generateTimer(timer: Timer): any {
    const jsonTimer: any = {};
    jsonTimer['name'] = timer.name;
    jsonTimer['period'] = Number(timer.period);

    return jsonTimer;
}

function generateBehavior(behavior: Behavior): any {
    const jsonBehavior: any = {};
    jsonBehavior['trigger'] = generateTrigger(behavior.trigger);

    return jsonBehavior;
}

function generateTrigger(trigger: Trigger): any {
    const jsonTrigger: any = {};
    jsonTrigger['type'] = trigger.$type;

    if (trigger.$type === 'MessageReceived') {
        jsonTrigger['value'] = trigger.topic;
    } else if (trigger.$type === 'TimerElapsed') {
        jsonTrigger['value'] = trigger.timer;
    } else if (trigger.$type === 'ServiceRequest') {
        jsonTrigger['value'] = trigger.service;
    } else if (trigger.$type === 'ActionGoalReceived') {
        jsonTrigger['value'] = trigger.action;
    } else if (trigger.$type === 'ParamChanged') {
        jsonTrigger['value'] = trigger.param;
    } else if (trigger.$type === 'StateChanged') {
        jsonTrigger['value'] = trigger.state;
    }

    return jsonTrigger;
}
