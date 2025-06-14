import { startLanguageServer } from 'langium/lsp';
import { NodeFileSystem } from 'langium/node';
import {
    createConnection,
    ProposedFeatures,
} from 'vscode-languageserver/node.js';
import { createRosAmlServices } from './ros-aml-module.js';

import fetch from 'node-fetch';
import { DocumentState } from 'langium';
import { Model, PredictedResult } from './generated/ast.js';
import { tabularModel } from './ast-processing/ast-to-tabular.js';
import crypto from 'crypto';

const connection = createConnection(ProposedFeatures.all);

const { shared } = createRosAmlServices({ connection, ...NodeFileSystem });

startLanguageServer(shared);



const lastSentHashes = new Map<string, string>();

function hashData(data: any): string {
    return crypto.createHash('sha256').update(JSON.stringify(data)).digest('hex');
}


shared.workspace.DocumentBuilder.onBuildPhase(DocumentState.Validated, async documents => {
    for (const document of documents) {
        const hasErrors = (document.diagnostics ?? []).some(d => d.severity === 1);
        if (hasErrors) {
            console.log(`Document ${document.uri} contains errors. AST not sent.`);
            continue; 
        }
        const model = document.parseResult.value as Model;
        const hasNodeWithBehavior = model.nodes?.some(node => node.behaviors && node.behaviors.length > 0);

        if (!hasNodeWithBehavior) {
            console.log(`Document ${document.uri} has no node with behaviors. AST not sent.`);
            continue;
        }

        const rows = tabularModel(model);
        const currentHash = hashData(rows);
        const uriString = document.uri.toString();
        if (lastSentHashes.get(uriString) === currentHash) {
            
            console.log(`Document ${uriString} content unchanged, skipping API call.`);
            continue;
        }


        try {
            const response = await fetch('http://localhost:5000/receive-ast', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(rows)
            });

            const predictions = await response.json() as number[];
            lastSentHashes.set(uriString, currentHash);

            if (!model.predictedResults) {
                model.predictedResults = [];
            }
            console.log("API response:", predictions);
            predictions.forEach((pred, index) => {
                const result: PredictedResult = {
                    $type: 'PredictedResult',
                    predictedTime: pred.toString(),
                    $container: model
                };
                model.predictedResults.push(result);
            });

            
        } catch (err) {
            console.error("Error sending AST:", err);
        }
    }
});


