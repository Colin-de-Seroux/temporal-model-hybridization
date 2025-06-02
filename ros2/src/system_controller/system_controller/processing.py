import rclpy
from rclpy.node import Node
import time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from system_controller.timer_execution import measure_execution_time
import json

class ProcessingNode(Node):
    def __init__(self):
        super().__init__('processing')
        self.declare_parameter('processingMode', 'normal')
        self.processState = 'waiting'


        self.subscription_sensorData = self.create_subscription(String, 'sensorData', self.sensorData_callback, 10)


    @measure_execution_time
    def on_messageReceived_sensorData(self):
        time.sleep(0.0005) 

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
    