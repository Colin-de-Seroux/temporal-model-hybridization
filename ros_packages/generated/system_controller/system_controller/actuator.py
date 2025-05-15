import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from system_controller.timer_execution import measure_execution_time
class ActuatorNode(Node):
    def __init__(self):
        super().__init__('Actuator')
        # Activation event-driven sur la source 'ProcessingNode'
        self.create_subscription(String, 'ProcessingNode', self.activation_callback, 10)
        self.declare_parameter('actuatorEnabled', 'true')
        self.actuatorStatus = 'off'


    @measure_execution_time()
    def on_messageReceived_processedData(self):
        self.set_parameters([rclpy.parameter.Parameter('actuatorEnabled', value='true')])
    
    def processedData_callback(self, msg):
        self.on_messageReceived_processedData()

    @measure_execution_time()
    def on_stateChanged_actuatorStatus(self):
        self.get_logger().debug('Actuator status updated')
    
    def on_state_changed(self):
        if self.actuatorStatus == True:
            self.on_stateChanged_actuatorStatus()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    