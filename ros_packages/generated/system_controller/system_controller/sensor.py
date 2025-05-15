import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from system_controller.timer_execution import measure_execution_time

class SensorNode(Node):
    def __init__(self):
        super().__init__('Sensor')
        self.declare_parameter('samplingRate', 10)
        self.sensorStatus = 'idle'
        self.readTimer_timer = self.create_timer(0.100, self.readTimer_callback)


    @measure_execution_time()
    def on_timerElapsed_readTimer(self):
        if not hasattr(self, 'publisher_sensorData'):
            self.publisher_sensorData = self.create_publisher(String, 'sensorData', 10)
        self.publisher_sensorData.publish(String(data='temperature_reading'))
    
    def readTimer_callback(self):
        self.on_timerElapsed_readTimer()

    @measure_execution_time()
    def on_paramChanged_samplingRate(self):
        self.get_logger().info("Sampling rate changed")
    
    def on_parameter_event(self, event):
        for changed_param in event.changed_parameters:
            if changed_param.name == 'samplingRate':
                self.on_paramChanged_samplingRate()


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    