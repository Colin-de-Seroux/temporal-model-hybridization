import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from callback.timer_execution import measure_execution_time


class testNodeNode(Node):
    def __init__(self):
        super().__init__('testNode')
        self.subscription_my_sub = self.create_subscription(String, 'my_sub', self.listener_callback_my_sub, 10)

        self.service_my_srv = self.create_service(example_interfaces/SetBool, 'my_srv', self.handle_my_srv)

        # Action server for my_act
        self.action_server_my_act = rclpy.action.ActionServer(
            self, DefaultActionType, 'my_act', self.execute_my_act)

        self.timer_cb_timer = self.create_timer(1, self.callback_cb_timer)

        self.subscription_my_sub = self.create_subscription(String, 'my_sub', self.callback_cb_sub, 10)

        self.service_my_srv = self.create_service(Empty, 'my_srv', self.callback_cb_srv)

        self.action_server_my_act = rclpy.action.ActionServer(self, Empty, 'my_act', self.callback_cb_act)

    def listener_callback_my_sub(self, msg):
        self.get_logger().info('data received')

    def handle_my_srv(self, request, response):
        # TODO: Implement service logic
        return response

    def execute_my_act(self, goal_handle):
        self.get_logger().info('Executing goal: goal data')
        # TODO: Add feedback publishing
        goal_handle.succeed()
        result = DefaultActionType.Result()
        result.result = "result data"
        return result

    @measure_execution_time()
    def callback_cb_timer(self):
        self.get_logger().info('Timer callback cb_timer triggered')

    @measure_execution_time()
    def callback_cb_sub(self, msg):
        self.get_logger().info('Subscriber callback cb_sub received: ' + str(msg.data))

    @measure_execution_time()
    def callback_cb_srv(self, request, response):
        self.get_logger().info('Service callback cb_srv triggered')
        return response

    @measure_execution_time()
    def callback_cb_act(self, goal_handle):
        self.get_logger().info('Action callback cb_act executing')
        goal_handle.succeed()
        result = Empty.Result()
        return result



def main(args=None):
    rclpy.init(args=args)

    node = testNodeNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
