import rclpy
from rclpy.node import Node
from timer_execution.timer_execution import measure_execution_time
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.create_timer(1.0, self.timer_callback)

    @measure_execution_time
    def my_heavy_function(self):
        time.sleep(0.3)  

    def timer_callback(self):
        self.my_heavy_function()

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()
