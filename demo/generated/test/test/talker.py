import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import *
from test.timer_execution import measure_execution_time

class talkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_chatter = self.create_publisher(string, 'chatter', 10)
        self.service_echo_service = self.create_service(AddTwoInts, 'echo_service', self.handle_echo_service)

        # Action server for nav_to_goal
        self.action_server_nav_to_goal = rclpy.action.ActionServer(
            self, NavigateToPose, 'nav_to_goal', self.execute_nav_to_goal)

    def handle_echo_service(self, request, response):
        # TODO: Implement service logic
        return response

    def execute_nav_to_goal(self, goal_handle):
        self.get_logger().info('Executing goal: Go to (1.0, 2.0)')
        # TODO: Add feedback publishing
        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.result = "Reached goal"
        return result



def main(args=None):
    rclpy.init(args=args)
    node = talkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
