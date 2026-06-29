import logging
import time


def get_logger(name: str = __name__) -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(name)s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


def log_error(context: str, error: Exception, recoverable: bool = True) -> None:
    logger = get_logger()
    logger.error(
        f"[{context}] {error.__class__.__name__}: {str(error)}",
        extra={
            'recoverable': recoverable,
            'timestamp': time.time()
        }
    )


def log_warning(context: str, message: str) -> None:
    logger = get_logger()
    logger.warning(f"[{context}] {message}")


def log_info(context: str, message: str) -> None:
    logger = get_logger()
    logger.info(f"[{context}] {message}")
