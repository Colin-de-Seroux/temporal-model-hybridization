import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from action_system.timer_execution import measure_execution_time
import json

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('actionserver')
        self.currentGoal = ''


        self._sendGoalAction_server = ActionServer(
            self,
            YOUR_ACTION_TYPE,  # TODO: Remplacer par le bon type
            'sendGoalAction',
            self.sendGoalAction_callback
        )


    @measure_execution_time
    def on_actionGoalReceived_sendGoalAction(self):
        self.get_logger().info("Action goal received")

        self.currentGoal = 'processing'


    def sendGoalAction_callback(self, goal_handle):
        goal= goal_handle.request
        self.on_actionGoalReceived_sendGoalAction()
        goal_handle.succeed()
        result = ... # TODO: définir le type et la réponse
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ActionServerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    