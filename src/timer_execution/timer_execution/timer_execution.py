import time
import functools
import rclpy
from rclpy.logging import get_logger

def measure_execution_time(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        start = time.time()
        result = func(*args, **kwargs)
        duration = (time.time() - start) * 1000  
        logger = get_logger('timer_execution')
        logger.info(f"Durée d'exécution de {func.__name__} : {duration:.2f} ms")
        return result
    return wrapper
