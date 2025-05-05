import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class Loop(Node):

    def __init__(self):
        super().__init__("loop")
        self.i = 0
        self.sum = 0
        self.logger = self.get_logger()
        self.test_loop()

    def test_loop(self):
        self.get_logger().info(f"<-->: {self.i + 1}")

        start = self.get_clock().now()

        j = 0

        while j < 50000:
            j += 1
        
        end = self.get_clock().now()
        duration_ms = (end - start).nanoseconds / 1e6
        self.sum += duration_ms
        self.logger.info(f"Duration: {duration_ms:.2f}ms")

        if self.i < 29:
            self.i += 1
            self.test_loop()
        else:
            self.logger.info("Loop finished")
            self.logger.info(f"Sum: {self.sum}")
            self.logger.info(f"Average: {self.sum / self.i:.2f}")


def main(args=None):
    rclpy.init(args=args)

    node = Loop()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()