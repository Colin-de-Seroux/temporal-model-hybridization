import functools
from rclpy.clock import Clock
from rclpy.logging import get_logger


def measure_execution_time(func):

    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        logger = get_logger("timer_execution")

        try:
            clock = Clock()
            start = clock.now()
            result = func(*args, **kwargs)
            end = clock.now()
            duration = (end - start).nanoseconds / 1e6
            logger.info(f"Time execution of function {func.__name__} : {duration:.2f} ms")

            return result
        except Exception as e:
            logger.error(f"Error in {func.__name__}: {e}")

            raise
            
    return wrapper