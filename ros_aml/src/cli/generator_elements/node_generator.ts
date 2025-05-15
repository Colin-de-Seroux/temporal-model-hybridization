import { Node } from '../../language/generated/ast.js';
import { CompositeGeneratorNode } from 'langium/generate';

export class NodeGenerator {
    private pkgName: string;
    private node: Node;
    private nodeBlock: CompositeGeneratorNode;
    private importBlock: CompositeGeneratorNode;
    private constructorBody: CompositeGeneratorNode;
    private methods: CompositeGeneratorNode;

    constructor(pkgName: string, node: Node) {
        this.pkgName = pkgName;
        this.node = node;
        this.nodeBlock = new CompositeGeneratorNode();
        this.importBlock = new CompositeGeneratorNode();
        this.constructorBody = new CompositeGeneratorNode();
        this.methods = new CompositeGeneratorNode();
    }

    public compile(): CompositeGeneratorNode {
        this.generateImports();
        this.generateClassHeader();
        this.generateConstructor();
        this.generateMethods();
        this.generateMainFunction();
        return this.nodeBlock;
    }

    private generateImports(): void {
        this.importBlock.append(`import rclpy`);
        this.importBlock.appendNewLine();
        this.importBlock.append(`from rclpy.node import Node`);
        this.importBlock.appendNewLine();
        this.importBlock.append(`from rclpy.executors import ExternalShutdownException`);
        this.importBlock.appendNewLine();
        this.importBlock.append(`from std_msgs.msg import String, Int32, Float32, Bool, Header`);
        this.importBlock.appendNewLine();
        this.importBlock.append(`from ${this.pkgName}.timer_execution import measure_execution_time`);
        this.importBlock.appendNewLine();
        this.importBlock.appendNewLine();
    }

    private generateClassHeader(): void {
        this.nodeBlock.append(`class ${this.node.name}Node(Node):`);
        this.nodeBlock.appendNewLine();
    }

    private generateConstructor(): void {
        this.constructorBody.append(`    def __init__(self):`);
        this.constructorBody.appendNewLine();
        this.constructorBody.append(`        super().__init__('${this.node.name}')`);
        this.constructorBody.appendNewLine();
    }

    private generateMethods(): void {
        this.nodeBlock.append(this.methods);
    }

    private generateMainFunction(): void {
        this.nodeBlock.appendNewLine();
        this.nodeBlock.append(`
def main(args=None):
    rclpy.init(args=args)

    node = ${this.node.name}Node()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
`);
    }
}
