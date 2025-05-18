import { type Model } from '../../language/generated/ast.js';

export function generateJsonGraphModel(model: Model): string {
    console.log(model);
    const graph: any = {};

    graph['name'] = model.systemName;
    graph['loggerLevel'] = model.logger.level;
    graph['nodes'] = [];

    model.nodes.forEach((node) => {
        const jsonNode: any = {};
        jsonNode['name'] = node.name;
        jsonNode['expectedExecTime'] = node.expectedExecTime;

        graph['nodes'].push(jsonNode);
    });

    const graphString = JSON.stringify(graph, null, 4);

    return graphString;
}
