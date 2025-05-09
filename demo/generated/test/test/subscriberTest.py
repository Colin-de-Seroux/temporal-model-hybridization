import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import *
from test.timer_execution import measure_execution_time

class subscriberTestNode(Node):
    def __init__(self):
        super().__init__('subscriberTest')
        self.publisher_pub = self.create_publisher(string, 'pub', 10)
        self.subscription_sub = self.create_subscription(string, 'sub', self.listener_callback_sub, 10)


def main(args=None):
    rclpy.init(args=args)
    node = subscriberTestNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
