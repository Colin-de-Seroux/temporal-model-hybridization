import {
    type Action,
    type Behavior,
    type Model,
} from '../../language/generated/ast.js';

export function generateJsonModel(model: Model): string {
    const graph: any = {};

    graph['name'] = model.systemName;
    graph['nodes'] = [];

    model.nodes.forEach((node) => {
        const jsonNode: any = {};
        jsonNode['name'] = node.name;
        jsonNode['expectedExecTime'] = Number(node.expectedExecTime);
        jsonNode['behaviors'] = [];

        const timers = [];

        for (const timer of node.timers) {
            timers.push({
                name: timer.name,
                period: Number(timer.period),
            });
        }

        for (const behavior of node.behaviors) {
            jsonNode['behaviors'].push(generateBehavior(behavior, timers));
        }

        graph['nodes'].push(jsonNode);
    });

    const graphString = JSON.stringify(graph, null, 4);

    return graphString;
}

function generateBehavior(
    behavior: Behavior,
    timers: { name: string; period: number }[]
): any {
    let jsonBehavior: any = { actions: [] };

    if (behavior.trigger.$type === 'TimerElapsed') {
        const trigger = behavior.trigger as { timer: string };
        jsonBehavior['actions'].push({
            type: 'timer',
            value: timers.find((t) => t.name === trigger.timer)?.period,
        });
    } else if (behavior.trigger.$type === 'MessageReceived') {
        jsonBehavior['actions'].push({
            type: 'sub',
            topic: behavior.trigger.topic,
        });
    }

    for (const action of behavior.action) {
        const jsonAction = generateAction(action);

        if (jsonAction !== undefined) {
            jsonBehavior['actions'].push(jsonAction);
        }
    }

    return jsonBehavior;
}

function generateAction(
    action: Action
): { type: string; topic?: string; value?: number } | undefined {
    if (action.$type === 'SendMessage') {
        return { type: 'pub', topic: action.topic };
    }

    if (action.$type === 'CallService' || action.$type === 'SendActionGoal') {
        return { type: 'other', value: Number(action.expectedTime) };
    }

    return undefined;
}
