import type { ValidationAcceptor, ValidationChecks } from 'langium';
import type { Model, Node, RosAmlAstType } from './generated/ast.js';
import type { RosAmlServices } from './ros-aml-module.js';

/**
 * Register custom validation checks.
 */
export function registerValidationChecks(services: RosAmlServices) {
    const registry = services.validation.ValidationRegistry;
    const validator = services.validation.RosAmlValidator;
    const checks: ValidationChecks<RosAmlAstType> = {
        Node: validator.checkPersonStartsWithCapital,
        Model: validator.checkModelPredictions
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
            if (firstChar.toUpperCase() !== firstChar) {
                accept('warning', 'Node name should start with an uppercase.', {
                    node: node,
                    property: 'name',
                });
            }
        }
    }

    
    checkModelPredictions(model: Model, accept: ValidationAcceptor): void {
        if (model.predictedResults && model.predictedResults.length > 0) {
            console.log(`Modèle "${model.systemName ?? 'inconnu'}" contient ${model.predictedResults.length} prédiction(s) :`);
            model.predictedResults.forEach((p, i) => {
                console.log(`  [${i}] predictedTime = ${p.predictedTime}`);
            });

            accept('info', `Ce modèle contient ${model.predictedResults.length} prédiction(s):  ${model.predictedResults.map(p => p.predictedTime).join(', ')}`, {
                node: model
            });
        }
        else {
            accept('warning', `Ce modèle ${model.predictedResults.length} ne contient aucune prédiction : ${model.predictedResults.map(p => p.predictedTime).join(', ')}`, {
                node: model
            });
        }
    }

    // validateTimerExecution(timer: Timer, accept: ValidationAcceptor) {
    //     const predicted = Number((timer as any).predictedPeriod);
    //     const period = Number(timer.period);
    //     if (!isNaN(predicted) && !isNaN(period) && Math.abs(predicted - period) > 10) {
    //         accept('warning', `The predicted period (${predicted}ms) differs significantly from the specified period (${period}ms).`, {
    //             node: timer,
    //             property: 'period'
    //         });
    //     }
    // }

}
