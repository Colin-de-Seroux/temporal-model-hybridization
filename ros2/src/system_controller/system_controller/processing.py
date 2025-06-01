import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from system_controller.timer_execution import measure_execution_time
import json

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing')
        self.declare_parameter('processingMode', 'normal')
        self.processState = 'waiting'

        self.client_dataProcessingService = self.create_client(ServiceType, 'dataProcessingService')

        self.subscription_sensorData = self.create_subscription(String, 'sensorData', self.sensorData_callback, 10)


    @measure_execution_time
    def on_messageReceived_sensorData(self):
        while not self.client_dataProcessingService.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting...')
        req = ServiceType.Request()
        req.data = json.dump({'data': 'temperature_reading'})
        future = self.client_dataProcessingService.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call succeeded')
        else:
            self.get_logger().error('Service call failed')


    def sensorData_callback(self, msg):
        self.on_messageReceived_sensorData()

    @measure_execution_time
    def on_serviceRequest_dataProcessingService(self):
        self.processState = 'processing'


    def dataProcessingService_callback(self, request, response):
        self.on_serviceRequest_dataProcessingService()
        response.success = True
        response.message = "Traitement lancé"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ProcessingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    