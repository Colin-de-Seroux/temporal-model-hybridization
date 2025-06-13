import {  isMessageReceived, isSendMessage, isTimerElapsed, Model } from "../generated/ast.js";

type TabularBehaviorAction = {
    modelName: string;
    nodeName: string;
    behaviorIndex: number;
    actionType: string;
    actionOrder: number;
    topic: string;
    value: number;
};

export function tabularModel(model: Model): TabularBehaviorAction[] {
    const data: TabularBehaviorAction[] = [];

    const modelName = model.systemName ?? 'UnnamedModel';

    model.nodes?.forEach((node, nodeIdx) => {
        const nodeName = node.name ?? `Node${nodeIdx}`;
        
        node.behaviors?.forEach((behavior, bIndex) => {
            if (isTimerElapsed(behavior.trigger)) {
                const timerName = behavior.trigger.timer;
                const timer = node.timers?.find(t => t.name === timerName);

                const period = timer?.period ?? '0.0';
                data.push({
                    modelName,
                    nodeName,
                    behaviorIndex: bIndex,
                    actionType: 'timer',
                    actionOrder: 0,
                    topic: '',
                    value: Number(period)
                });
            }
            else if (isMessageReceived(behavior.trigger)) {
                data.push({
                    modelName,
                    nodeName,
                    behaviorIndex: bIndex,
                    actionType: 'sub',
                    actionOrder: 0,
                    topic: behavior.trigger.topic ?? '',
                    value: 0.0
                });
            }

            behavior.action?.forEach((action, aIndex) => {
                if (isSendMessage(action)) {
                    data.push({
                        modelName,
                        nodeName,
                        behaviorIndex: bIndex,
                        actionType: 'pub',
                        actionOrder: aIndex ,
                        topic: action.topic ?? '',
                        value: 0.0
                    });
                } 
            });
        });
    });

    return data;
}

