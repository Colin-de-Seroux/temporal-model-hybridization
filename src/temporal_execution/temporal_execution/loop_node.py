import rclpy
from rclpy.node import Node
from temporal_execution.timer_execution import measure_execution_time


class LoopNode(Node):

    def __init__(self, i: int):
        super().__init__(f"loop{i}")

    @measure_execution_time
    def test_loop(self) -> int:
        """
        This function simulates a loop that runs 100,000 iterations.
        It is decorated with the measure_execution_time decorator to log
        the execution time.

        :return: The final value of j after the loop completes.
        :rtype: int
        """

        j = 0

        while j < 100000:
            j += 1
        
        return j


@measure_execution_time  
def execution_nodes(nodes: list):
    """
    This function executes the test_loop method for a list of nodes.

    :param nodes: A list of LoopNode instances.
    :type nodes: list
    """

    for node in nodes:
        node.test_loop()


@measure_execution_time
def main(args=None):
    rclpy.init(args=args)

    nodes = []
    nb_nodes = 30

    for i in range(nb_nodes):
        node = LoopNode(i)
        nodes.append(node)
    
    execution_nodes(nodes)

    for node in nodes:
        node.destroy_node()

    rclpy.shutdown()
