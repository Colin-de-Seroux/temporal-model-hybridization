import { Behavior, Value } from "../../language/generated/ast.js";

export function inferTypeFromMessage(msg: string): string {
    if (/^-?\d+$/.test(msg)) return 'std_msgs.msg.Int32';
    if (/^-?\d+\.\d+$/.test(msg)) return 'std_msgs.msg.Float64';
    if (/^["'].*["']$/.test(msg)) return 'std_msgs.msg.String';
    return 'std_msgs.msg.String';
}
export function resolveMessageType(
    userType: string | undefined,
    message: string
): string {
    const typeName = userType?.trim() || inferTypeFromMessage(message);
    return typeName.split('.').at(-1) ?? typeName;
}

export function getBehaviorMethodName(behavior: Behavior): string {
    if (behavior.trigger.timer) return `on_timerElapsed_${behavior.trigger.timer}`;
    if (behavior.trigger.topic) return `on_messageReceived_${behavior.trigger.topic}`;
    if (behavior.trigger.service) return `on_serviceRequest_${behavior.trigger.service}`;
    if (behavior.trigger.action) return `on_actionGoalReceived_${behavior.trigger.action}`;
    if (behavior.trigger.param) return `on_paramChanged_${behavior.trigger.param}`;
    if (behavior.trigger.state) return `on_stateChanged_${behavior.trigger.state}`;
    return 'on_unknown_trigger';
}

export function generateValueCode(value: Value | undefined): string {
    if (!value) return 'None'; 

    if (value.intValue !== undefined) return value.intValue.toString();
    if (value.doubleValue !== undefined) return value.doubleValue.toString();
    if (value.stringValue !== undefined) return `'${value.stringValue}'`; 
    if (value.boolValue !== undefined) return value.boolValue === 'true' ? 'True' : 'False';

    return 'None'; 
}