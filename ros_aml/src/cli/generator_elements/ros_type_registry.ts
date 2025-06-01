import { execSync } from 'child_process';

export interface RosFieldType {
    name: string;
    type: string;
    isArray: boolean;
}

export interface RosTypeDefinition {
    name: string;
    fields: RosFieldType[];
    isService?: boolean;
    requestFields?: RosFieldType[];
    responseFields?: RosFieldType[];
}

export class RosTypeRegistry {
    private messageTypes: Map<string, RosTypeDefinition> = new Map();
    private serviceTypes: Map<string, RosTypeDefinition> = new Map();

    constructor() {
        this.loadMessageTypes();
        this.loadServiceTypes();
    }

    private runCommand(command: string): string {
        return execSync(command, { encoding: 'utf-8' });
    }

    private loadMessageTypes(): void {
        const rawTypes = this.runCommand('ros2 interface list -m').split('\n');
        for (const type of rawTypes) {
            const trimmed = type.trim();
            if (trimmed) {
                const definition = this.parseInterface(trimmed, false);
                if (definition) this.messageTypes.set(trimmed, definition);
            }
        }
    }

    private loadServiceTypes(): void {
        const rawTypes = this.runCommand('ros2 interface list -s').split('\n');
        for (const type of rawTypes) {
            const trimmed = type.trim();
            if (trimmed) {
                const definition = this.parseInterface(trimmed, true);
                if (definition) this.serviceTypes.set(trimmed, definition);
            }
        }
    }

    private parseInterface(name: string, isService: boolean): RosTypeDefinition | undefined {
        try {
            const raw = this.runCommand(`ros2 interface show ${name}`);
            const parts = raw.split('---');
            if (isService && parts.length === 2) {
                return {
                    name,
                    isService: true,
                    requestFields: this.parseFields(parts[0]),
                    responseFields: this.parseFields(parts[1]),
                    fields: [], // Not used for services
                };
            } else if (!isService) {
                return {
                    name,
                    isService: false,
                    fields: this.parseFields(raw),
                };
            }
        } catch (e) {
            console.warn(`Failed to parse interface ${name}: ${e}`);
        }
        return undefined;
    }

    private parseFields(definition: string): RosFieldType[] {
        const lines = definition.split('\n');
        const fields: RosFieldType[] = [];

        for (const line of lines) {
            const match = line.trim().match(/^([\w\/\[\]]+)\s+(\w+)/);
            if (match) {
                const typeStr = match[1];
                const isArray = typeStr.endsWith('[]');
                fields.push({
                    type: isArray ? typeStr.slice(0, -2) : typeStr,
                    name: match[2],
                    isArray,
                });
            }
        }

        return fields;
    }

    getMessageType(name: string): RosTypeDefinition | undefined {
        return this.messageTypes.get(name);
    }

    getServiceType(name: string): RosTypeDefinition | undefined {
        return this.serviceTypes.get(name);
    }

    hasMessageType(name: string): boolean {
        return this.messageTypes.has(name);
    }

    hasServiceType(name: string): boolean {
        return this.serviceTypes.has(name);
    }
}
