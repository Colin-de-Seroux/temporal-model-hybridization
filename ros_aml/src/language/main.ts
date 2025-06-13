import { startLanguageServer } from 'langium/lsp';
import { NodeFileSystem } from 'langium/node';
import {
    createConnection,
    ProposedFeatures,
} from 'vscode-languageserver/node.js';
import { createRosAmlServices } from './ros-aml-module.js';

import fetch from 'node-fetch';
import { DocumentState } from 'langium';
import { Model } from './generated/ast.js';

// Create a connection to the client
const connection = createConnection(ProposedFeatures.all);

// Inject the shared services and language-specific services
const { shared , RosAml} = createRosAmlServices({ connection, ...NodeFileSystem });

// Start the language server with the shared services
startLanguageServer(shared);

// type DocumentChange = { uri: string, content: string, diagnostics: Diagnostic[] };

const JsonSerializer = RosAml.serializer.JsonSerializer;
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

        const jsonAst = JsonSerializer.serialize(model, {
            sourceText: false,
            textRegions: false
        });

        try {
            const response = await fetch('http://localhost:5000/receive-ast', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(jsonAst)
            });

            const result = await response.json();
            console.log("API response:", result);
        } catch (err) {
            console.error("Error sending AST:", err);
        }
    }
});
