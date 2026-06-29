import pytest
import logging
from optimize_stitching.logger import get_logger, log_error, log_warning, log_info


class TestLogger:
    def test_get_logger_returns_logger(self):
        logger = get_logger("test")
        assert isinstance(logger, logging.Logger)
        assert logger.name == "test"

    def test_get_logger_singleton(self):
        logger1 = get_logger("test_singleton")
        logger2 = get_logger("test_singleton")
        assert logger1 is logger2

    def test_logger_level(self):
        logger = get_logger("test_level")
        assert logger.level == logging.INFO

    def test_logger_handler_exists(self):
        logger = get_logger("test_handler")
        assert len(logger.handlers) > 0

    def test_logger_formatter(self):
        logger = get_logger("test_formatter")
        handler = logger.handlers[0]
        assert isinstance(handler.formatter, logging.Formatter)
        assert '%(levelname)' in handler.formatter._fmt
        assert '%(message)' in handler.formatter._fmt

    def test_log_info_runs_without_error(self):
        log_info("TestContext", "Test message")

    def test_log_warning_runs_without_error(self):
        log_warning("TestContext", "Test warning")

    def test_log_error_runs_without_error(self):
        try:
            raise ValueError("test error")
        except ValueError as e:
            log_error("TestContext", e)

    def test_log_error_with_recoverable_flag(self):
        try:
            raise ValueError("test error")
        except ValueError as e:
            log_error("TestContext", e, recoverable=True)

    def test_log_error_with_unrecoverable_flag(self):
        try:
            raise ValueError("test error")
        except ValueError as e:
            log_error("TestContext", e, recoverable=False)
