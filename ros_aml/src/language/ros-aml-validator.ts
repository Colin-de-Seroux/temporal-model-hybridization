import type { ValidationAcceptor, ValidationChecks } from 'langium';
import type { RosAmlAstType, Person } from './generated/ast.js';
import type { RosAmlServices } from './ros-aml-module.js';

/**
 * Register custom validation checks.
 */
export function registerValidationChecks(services: RosAmlServices) {
    const registry = services.validation.ValidationRegistry;
    const validator = services.validation.RosAmlValidator;
    const checks: ValidationChecks<RosAmlAstType> = {
        Person: validator.checkPersonStartsWithCapital
    };
    registry.register(checks, validator);
}

/**
 * Implementation of custom validations.
 */
export class RosAmlValidator {

    checkPersonStartsWithCapital(person: Person, accept: ValidationAcceptor): void {
        if (person.name) {
            const firstChar = person.name.substring(0, 1);
            if (firstChar.toUpperCase() !== firstChar) {
                accept('warning', 'Person name should start with a capital.', { node: person, property: 'name' });
            }
        }
    }

}
