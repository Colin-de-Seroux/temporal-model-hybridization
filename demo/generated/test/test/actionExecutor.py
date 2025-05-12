import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from test.timer_execution import measure_execution_time

class actionExecutorNode(Node):
    def __init__(self):
        super().__init__('actionExecutor')
        # Action server for nav_to_goal
        self.action_server_nav_to_goal = rclpy.action.ActionServer(
            self, NavigateToPose, 'nav_to_goal', self.execute_nav_to_goal)

    def execute_nav_to_goal(self, goal_handle):
        self.get_logger().info('Executing goal: Start navigation')
        # TODO: Add feedback publishing
        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.result = "Navigation done"
        return result



def main(args=None):
    rclpy.init(args=args)
    node = actionExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
