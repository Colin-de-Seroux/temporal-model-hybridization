/*import { CompositeGeneratorNode } from 'langium/generate';
import {
    Node,
} from '../../language/generated/ast.js';

class ImportGenerator {
    constructor(private pkgName: string, private node: Node) {
        this.pkgName = pkgName;
        this.node = node;
    }

    public generate(): CompositeGeneratorNode {
        const imports = new CompositeGeneratorNode();
        imports.append(`import rclpy`);
        imports.appendNewLine();
        imports.append(`from rclpy.node import Node`);
        imports.appendNewLine();
        imports.append(`from ${this.pkgName}.timer_execution import measure_execution_time`);
        imports.appendNewLine();

        /*if (this.node.behaviors && this.node.behaviors.some(behavior => /* your condition here )) {
            imports.append(`from std_msgs.msg import String`);
            imports.appendNewLine();
        }

        return imports;
    }
}*/
