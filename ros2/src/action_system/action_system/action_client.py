import rclpy
from rclpy.node import Node
import time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from action_system.timer_execution import measure_execution_time
import json
from datetime import datetime

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('actionclient')
        self.periodicTimer_timer = self.create_timer(1.000, self.periodicTimer_callback)



    @measure_execution_time
    def on_timerElapsed_periodicTimer(self):
        time.sleep(0.003) 

    def periodicTimer_callback(self):
        self.on_timerElapsed_periodicTimer()


def main(args=None):
    rclpy.init(args=args)
    node = ActionClientNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    