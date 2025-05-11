export function inferTypeFromMessage(msg: string): string {
    if (/^-?\d+$/.test(msg)) return 'std_msgs.msg.Int32';
    if (/^-?\d+\.\d+$/.test(msg)) return 'std_msgs.msg.Float64';
    if (/^["'].*["']$/.test(msg)) return 'std_msgs.msg.String';
    return 'std_msgs.msg.String'; 
}
export function resolveMessageType(userType: string | undefined, message: string): string {
    const typeName = userType?.trim() || inferTypeFromMessage(message);
    return typeName.split('.').at(-1) ?? typeName;
}