import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from service_system.timer_execution import measure_execution_time
import json

class ServiceHandlerNode(Node):
    def __init__(self):
        super().__init__('servicehandler')
        self.declare_parameter('use_led', 'true')
        self.led_state = 'false'
        self.blink_timer_timer = self.create_timer(1.000, self.blink_timer_callback)

        self.publisher_led_status = self.create_publisher(String, 'led_status', 10)
        self.client_check_battery = self.create_client(ServiceType, 'check_battery')


    @measure_execution_time()
    def on_serviceRequest_toggle_led(self):
        self.get_logger().info("Received toggle request")

        self.led_state = 'true'

        self.publisher_led_status.publish(String(data='LED toggled'))


    def toggle_led_callback(self, request, response):
        self.on_serviceRequest_toggle_led()
        response.success = True
        response.message = "Traitement lancé"
        return response

    @measure_execution_time()
    def on_timerElapsed_blink_timer(self):
        self.get_logger().debug("Checking battery status")

        while not self.client_check_battery.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')
        req = ServiceType.Request()
        req.data = json.dump({'threshold': 20})
        future = self.client_check_battery.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call succeeded')
        else:
            self.get_logger().error('Service call failed')


    def blink_timer_callback(self):
        self.on_timerElapsed_blink_timer()

    @measure_execution_time()
    def on_paramChanged_use_led(self):
        self.get_logger().info("Parameter 'use_led' changed")


    def on_parameter_event(self, event):
        for changed_param in event.changed_parameters:
            if changed_param.name == 'use_led':
                self.on_paramChanged_use_led()


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
    