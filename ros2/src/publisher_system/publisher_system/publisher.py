import rclpy
from rclpy.node import Node
import time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from publisher_system.timer_execution import measure_execution_time
import json

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')
        self.declare_parameter('rate', 1000)
        self.mode = 'idle'
        self.pub_timer_timer = self.create_timer(1.000, self.pub_timer_callback)

        self.publisher_data_topic = self.create_publisher(String, 'data_topic', 10)


    @measure_execution_time
    def on_timerElapsed_pub_timer(self):
        self.publisher_data_topic.publish(String(data='Hello from publisher'))


    def pub_timer_callback(self):
        self.on_timerElapsed_pub_timer()


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
    