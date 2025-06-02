import { type Model, type Timer } from '../../language/generated/ast.js';

export function generateJsonModel(model: Model): string {
    const graph: any = {};

    graph['name'] = model.systemName;
    graph['nodes'] = [];

    model.nodes.forEach((node) => {
        const jsonNode: any = {};
        jsonNode['name'] = node.name;
        jsonNode['expectedExecTime'] = Number(node.expectedExecTime);

        for (const timer of node.timers) {
            jsonNode['timer'] = generateTimer(timer);
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
