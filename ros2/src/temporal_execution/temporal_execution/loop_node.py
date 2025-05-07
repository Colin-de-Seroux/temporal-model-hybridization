import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from temporal_execution.timer_execution import measure_execution_time

from std_msgs.msg import Int32

class LoopNode(Node):

    def __init__(self):
        super().__init__(f"loop")

        self.pub = self.create_publisher(Int32, "loop", 10)
        timer_period = 0.1
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    @measure_execution_time
    def timer_callback(self):
        """
        This function simulates a loop that runs 1 000 000 iterations.
        It is decorated with the measure_execution_time decorator to log
        the execution time.
        """

        j = 0

        while j < 1000000:
            j += 1
        
        msg = Int32()
        msg.data = j
        
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = LoopNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
