import type { ValidationAcceptor, ValidationChecks } from 'langium';
import type { Node, RosAmlAstType } from './generated/ast.js';
import type { RosAmlServices } from './ros-aml-module.js';

/**
 * Register custom validation checks.
 */
export function registerValidationChecks(services: RosAmlServices) {
    const registry = services.validation.ValidationRegistry;
    const validator = services.validation.RosAmlValidator;
    const checks: ValidationChecks<RosAmlAstType> = {
        Node: validator.checkPersonStartsWithCapital,
    };
    registry.register(checks, validator);
}

/**
 * Implementation of custom validations.
 */
export class RosAmlValidator {
    checkPersonStartsWithCapital(node: Node, accept: ValidationAcceptor): void {
        if (node.name) {
            const firstChar = node.name.substring(0, 1);
            if (firstChar.toLowerCase() !== firstChar) {
                accept('warning', 'Node name should start with a minuscule.', {
                    node: node,
                    property: 'name',
                });
            }
        }
    }
}
