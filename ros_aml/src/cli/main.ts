import type { Model } from '../language/generated/ast.js';
import chalk from 'chalk';
import { Command } from 'commander';
import { RosAmlLanguageMetaData } from '../language/generated/module.js';
import { createRosAmlServices } from '../language/ros-aml-module.js';
import { extractAstNode } from './cli-util.js';
import { generateRosScript } from './generator.js';
import { NodeFileSystem } from 'langium/node';
import * as url from 'node:url';
import * as fs from 'node:fs/promises';
import * as path from 'node:path';
const __dirname = url.fileURLToPath(new URL('.', import.meta.url));

const packagePath = path.resolve(__dirname, '..', '..', 'package.json');
const packageContent = await fs.readFile(packagePath, 'utf-8');

export const generateAction = async (
    fileName: string,
    opts: GenerateOptions
): Promise<void> => {
    const services = createRosAmlServices(NodeFileSystem).RosAml;
    const model = await extractAstNode<Model>(fileName, services);
    const generatedFilePath = generateRosScript(
        model,
        fileName,
        opts.destination
    );
    console.log(
        chalk.green(`ROS2 code generated successfully: ${generatedFilePath}`)
    );
};

export const generateAllAction = async (
    opts: GenerateOptions & { dir?: string }
): Promise<void> => {
    const ros_configs = path.resolve(__dirname, '..', '..', '..','ros_configs');

    const files = await fs.readdir(ros_configs);
    const rosamlFiles = files.filter(f => f.endsWith('.rosaml'));

    if (rosamlFiles.length === 0) {
        console.log(chalk.yellow(`⚠️ No .rosaml files found in ${ros_configs}`));
        return;
    }

    for (const file of rosamlFiles) {
        const filePath = path.join(ros_configs, file);
        console.log(chalk.cyan(`\tGenerating: ${filePath}`));
        await generateAction(filePath, opts);
    }

    console.log(chalk.green(`✅ All .rosaml files in ${ros_configs} processed.`));
};

export type GenerateOptions = {
    destination?: string;
};

export default function (): void {
    const program = new Command();

    program.version(JSON.parse(packageContent).version);

    const fileExtensions = RosAmlLanguageMetaData.fileExtensions.join(', ');
    program
        .command('generate')
        .argument(
            '[file]',
            `source file (possible file extensions: ${fileExtensions})`
        )
        .option('--all', 'generate all .rosaml files in a directory')
        .option(
            '-d, --destination <dir>',
            'destination directory of generating'
        )
        .description('generates ROS 2 code and package')
        .action(async (file: string | undefined, opts: any) => {
            if (opts.all) {
                await generateAllAction(opts);
            } else if (file) {
                await generateAction(file, opts);
            } else {
                console.error(chalk.red('❌ Please provide a file or use --all.'));
                process.exit(1);
            }
        });

    program.parse(process.argv);
}
