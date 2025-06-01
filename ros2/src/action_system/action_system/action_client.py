import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from action_system.timer_execution import measure_execution_time
import json

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('actionclient')
        self.periodicTimer_timer = self.create_timer(1.000, self.periodicTimer_callback)

        self.action_client_sendGoalAction = ActionClient(self, sendGoalAction, 'sendGoalAction')


    @measure_execution_time()
    def on_timerElapsed_periodicTimer(self):
        goal_msg = 'start'
        self.action_client_sendGoalAction.send_goal_async(goal_msg)


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
    