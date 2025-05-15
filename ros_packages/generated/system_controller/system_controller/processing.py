import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from system_controller.timer_execution import measure_execution_time
class ProcessingNode(Node):
    def __init__(self):
        super().__init__('Processing')
        # Activation event-driven sur la source 'SensorNode'
        self.create_subscription(String, 'SensorNode', self.activation_callback, 10)
        self.declare_parameter('processingMode', 'normal')
        self.processState = 'waiting'


    @measure_execution_time()
    def on_messageReceived_sensorData(self):
        # Appel de service dataProcessingService
        self.get_logger().info("callService not implemented yet")
    
    def sensorData_callback(self, msg):
        self.on_messageReceived_sensorData()

    @measure_execution_time()
    def on_serviceRequest_dataProcessingService(self):
        self.processState = 'processing'
    
    def dataProcessingService_callback(self, request, response):
        self.on_serviceRequest_dataProcessingService()
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
    