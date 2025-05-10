import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import *
from test.timer_execution import measure_execution_time

class serviceHandlerNode(Node):
    def __init__(self):
        super().__init__('serviceHandler')
        self.service_echo_service = self.create_service(AddTwoInts, 'echo_service', self.handle_echo_service)

    def handle_echo_service(self, request, response):
        # TODO: Implement service logic
        return response



def main(args=None):
    rclpy.init(args=args)
    node = serviceHandlerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
