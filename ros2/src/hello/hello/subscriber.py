import rclpy
from rclpy.node import Node
import time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from hello.timer_execution import measure_execution_time
import json
from datetime import datetime

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber')


        self.subscription_hello_topic = self.create_subscription(String, 'hello_topic', self.hello_topic_callback, 10)


    @measure_execution_time
    def on_messageReceived_hello_topic(self):
        self.get_logger().info("I heard: hello")


    def hello_topic_callback(self, msg):
        finish_time = time.time()
        self.get_logger().info("Received at: " + str(finish_time))
        self.on_messageReceived_hello_topic()


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    