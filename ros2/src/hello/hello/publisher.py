import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from hello.timer_execution import measure_execution_time
import json

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')
        self.publish_timer_timer = self.create_timer(1.000, self.publish_timer_callback)

        self.publisher_hello_topic = self.create_publisher(String, 'hello_topic', 10)


    @measure_execution_time()
    def on_timerElapsed_publish_timer(self):
        self.publisher_hello_topic.publish(String(data='hello'))


    def publish_timer_callback(self):
        self.on_timerElapsed_publish_timer()


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    