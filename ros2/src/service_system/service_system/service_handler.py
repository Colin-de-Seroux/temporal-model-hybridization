import rclpy
from rclpy.node import Node
import time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from service_system.timer_execution import measure_execution_time
import json
from datetime import datetime

class ServiceHandlerNode(Node):
    def __init__(self):
        super().__init__('servicehandler')
        self.blink_timer_timer = self.create_timer(1.000, self.blink_timer_callback)



    @measure_execution_time
    def on_timerElapsed_blink_timer(self):
        self.get_logger().debug("Checking service status")

        time.sleep(0.0005) 

    def blink_timer_callback(self):
        self.on_timerElapsed_blink_timer()


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
    