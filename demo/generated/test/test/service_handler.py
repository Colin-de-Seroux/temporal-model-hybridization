import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from test.timer_execution import measure_execution_time


class ServiceHandlerNode(Node):
    def __init__(self):
        super().__init__('ServiceHandler')
        self.service_echo_service = self.create_service(AddTwoInts, 'echo_service', self.handle_echo_service)

    def handle_echo_service(self, request, response):
        # TODO: Implement service logic
        return response



def main(args=None):
    rclpy.init(args=args)

    node = ServiceHandlerNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
