import rclpy
from rclpy.node import Node
import time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from pub_and_sub.timer_execution import measure_execution_time
import json
from datetime import datetime

class PubSubNode(Node):
    def __init__(self):
        super().__init__('pubsub')
        self.heartbeat_timer = self.create_timer(1.000, self.heartbeat_callback)


        self.subscription_topic = self.create_subscription(String, 'topic', self.topic_callback, 10)
        self.publisher_topic = self.create_publisher(String, 'topic', 10)


    @measure_execution_time
    def on_messageReceived_topic(self):
        self.get_logger().info("Received sensor data")


    def topic_callback(self, msg):
        finish_time = time.time()
        self.get_logger().info("Received at: " + str(finish_time))
        self.on_messageReceived_topic()

    @measure_execution_time
    def on_timerElapsed_heartbeat(self):
        self.get_logger().debug("Heartbeat timer elapsed")

        start_time = time.time()
        self.get_logger().info("Send at: " + str(start_time))
        self.publisher_topic.publish(String(data='ping'))


    def heartbeat_callback(self):
        self.on_timerElapsed_heartbeat()


def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    