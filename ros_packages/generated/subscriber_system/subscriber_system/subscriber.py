import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from subscriber_system.timer_execution import measure_execution_time

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.msg_timer_timer = self.create_timer(1.000, self.msg_timer_callback)


        self.subscription_data_topic = self.create_subscription(String, 'data_topic', self.data_topic_callback, 10)


    @measure_execution_time()
    def on_messageReceived_data_topic(self):
        self.get_logger().info("Message received on data_topic")


    def data_topic_callback(self, msg):
        self.on_messageReceived_data_topic()


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
    