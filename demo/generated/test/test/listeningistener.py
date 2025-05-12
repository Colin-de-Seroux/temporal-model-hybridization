import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from test.timer_execution import measure_execution_time

class listeningistenerNode(Node):
    def __init__(self):
        super().__init__('listeningistener')
        self.publisher_listening_feedback = self.create_publisher(String, 'listening_feedback', 10)
        self.subscription_chatter = self.create_subscription(String, 'chatter', self.listener_callback_chatter, 10)

    def listener_callback_chatter(self, msg):
        self.get_logger().info('I heard something')



def main(args=None):
    rclpy.init(args=args)
    node = listeningistenerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
