import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.logging import get_logger


class Loop(Node):

    def __init__(self, i):
        super().__init__(f"loop{i}")

    def test_loop(self):
        j = 0

        while j < 100000:
            j += 1
        
        return j


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("loop_node")

    nodes = []
    nb_nodes = 30

    start = Clock().now().nanoseconds

    for i in range(nb_nodes):
        node = Loop(i)
        nodes.append(node)
    
    end = Clock().now().nanoseconds
    ms_creation = (end - start) / 1e6
    logger.info(f"Global creation time in {ms_creation / nb_nodes:2f} milliseconds")

    start = Clock().now().nanoseconds

    for node in nodes:
        node.test_loop()

    end = Clock().now().nanoseconds
    ms_execution = (end - start) / 1e6
    logger.info(f"Global execution time in {ms_execution / nb_nodes:2f} milliseconds")

    start = Clock().now().nanoseconds
    
    for node in nodes:
        node.destroy_node()

    end = Clock().now().nanoseconds
    ms_destruction = (end - start) / 1e6
    logger.info(f"Global destruction time in {ms_destruction / nb_nodes:2f} milliseconds")
    logger.info(f"Global time in {(ms_creation + ms_execution + ms_destruction) / nb_nodes:2f} milliseconds")
    
    rclpy.shutdown()
