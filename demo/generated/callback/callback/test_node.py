import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from callback.timer_execution import measure_execution_time


class testNodeNode(Node):
    def __init__(self):
        super().__init__('testNode')
        self.timer_cb_timer = self.create_timer(1, self.callback_cb_timer)

        self.subscription_cb_sub = self.create_subscription(String, 'cb_sub_topic', self.callback_cb_sub, 10.0)

        self.service_cb_srv = self.create_service(Empty, 'cb_srv_service', self.callback_cb_srv)

        self.action_server_cb_act = rclpy.action.ActionServer(self, Empty, 'cb_act_action', self.callback_cb_act)

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
