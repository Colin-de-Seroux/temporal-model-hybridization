import rclpy
from rclpy.node import Node
import time
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Int32, Float32, Bool, Header
from publish_subscribe_node.timer_execution import measure_execution_time
import json
from datetime import datetime

class PubSubNode(Node):
    def __init__(self):
        super().__init__('pubsub')
        self.declare_parameter('use_feature', 'true')
        self.system_mode = 'idle'
        self.heartbeat_timer = self.create_timer(1.000, self.heartbeat_callback)


        self.subscription_sensor_data = self.create_subscription(String, 'sensor_data', self.sensor_data_callback, 10)
        self.publisher_heartbeat_topic = self.create_publisher(String, 'heartbeat_topic', 10)


    @measure_execution_time
    def on_messageReceived_sensor_data(self):
        self.get_logger().info("Received sensor data")

        self.system_mode = 'active'


    def sensor_data_callback(self, msg):
        finish_time = time.time()
        self.get_logger().info("Received at: " + str(finish_time))
        self.on_messageReceived_sensor_data()

    @measure_execution_time
    def on_timerElapsed_heartbeat(self):
        self.get_logger().debug("Heartbeat timer elapsed")

        start_time = time.time()
        self.get_logger().info("Send at: " + str(start_time))
        self.publisher_heartbeat_topic.publish(String(data='ping'))


    def heartbeat_callback(self):
        self.on_timerElapsed_heartbeat()

    @measure_execution_time
    def on_paramChanged_use_feature(self):
        self.get_logger().info("Parameter use_feature changed")


    def on_parameter_event(self, event):
        for changed_param in event.changed_parameters:
            if changed_param.name == 'use_feature':
                self.on_paramChanged_use_feature()

    @measure_execution_time
    def on_stateChanged_system_mode(self):
        self.get_logger().info("System mode changed")


    def on_state_changed(self):
        if self.system_mode == True:
            self.on_stateChanged_system_mode()


def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    