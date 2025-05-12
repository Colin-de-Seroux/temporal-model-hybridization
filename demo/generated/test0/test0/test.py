import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from test0.timer_execution import measure_execution_time


class TestNode(Node):
    def __init__(self):
        super().__init__('Test')
        self.get_logger().info('Test was launch')


def main(args=None):
    rclpy.init(args=args)

    node = TestNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
