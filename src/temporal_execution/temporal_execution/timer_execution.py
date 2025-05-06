import functools
from rclpy.clock import Clock
from rclpy.logging import get_logger


def measure_execution_time(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        try:
            clock = Clock() 
            start = clock.now()
            result = func(*args, **kwargs)
            end = clock.now()
            duration = (end - start).nanoseconds / 1e6 
            logger = get_logger('timer_execution')
            logger.info(f"Durée d'exécution de {func.__name__} : {duration:.2f} ms")
            return result
        except Exception as e:
            logger = get_logger('timer_execution')
            logger.error(f"Erreur dans {func.__name__}: {e}")
            raise
    return wrapper

   