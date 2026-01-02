"""
Centralized logging configuration for the Humanoid Robotics Book backend
Sets up default logging options and provides configuration presets
"""

import os
from typing import Optional
from backend.log_utils import StructuredLogger, logger as default_logger

# Define configuration presets for different environments
LOGGING_CONFIGS = {
    "development": {
        "level": "DEBUG",
        "log_file": "logs/app_dev.log",
        "max_bytes": 10 * 1024 * 1024,  # 10MB
        "backup_count": 3,
        "include_console": True,
        "remote_endpoint": None,
    },
    "production": {
        "level": "INFO",
        "log_file": "logs/app_prod.log",
        "max_bytes": 50 * 1024 * 1024,  # 50MB
        "backup_count": 10,
        "include_console": True,
        "remote_endpoint": os.getenv("LOGGING_ENDPOINT", "https://example.com/api/logs"),
    },
    "testing": {
        "level": "DEBUG",
        "log_file": "logs/app_test.log",
        "max_bytes": 5 * 1024 * 1024,  # 5MB
        "backup_count": 2,
        "include_console": True,
        "remote_endpoint": None,
    }
}

def get_logging_config(env: Optional[str] = None) -> dict:
    """Get logging configuration for the specified environment"""
    if env is None:
        env = os.getenv("ENVIRONMENT", "development")
    
    config = LOGGING_CONFIGS.get(env, LOGGING_CONFIGS["development"])
    
    # Convert level string to logging constant
    import logging
    level_map = {
        "DEBUG": logging.DEBUG,
        "INFO": logging.INFO,
        "WARNING": logging.WARNING,
        "ERROR": logging.ERROR,
        "CRITICAL": logging.CRITICAL
    }
    config["level"] = level_map[config["level"]]
    
    return config

def configure_logging(env: Optional[str] = None, custom_config: Optional[dict] = None) -> StructuredLogger:
    """Configure and return a logger instance with the specified settings"""
    config = get_logging_config(env)
    
    # Override with custom config if provided
    if custom_config:
        config.update(custom_config)
    
    # Create a new logger instance with the configuration
    logger = StructuredLogger(**config)
    
    return logger

def log_event(event: str, context: Optional[dict] = None, component: Optional[str] = None):
    """Log an event with standard format"""
    default_logger.info(f"Event: {event}", context, component)

def log_error(error: str, context: Optional[dict] = None, component: Optional[str] = None):
    """Log an error with standard format"""
    default_logger.error(error, context, component)

def log_warning(warning: str, context: Optional[dict] = None, component: Optional[str] = None):
    """Log a warning with standard format"""
    default_logger.warning(warning, context, component)

def log_debug(message: str, context: Optional[dict] = None, component: Optional[str] = None):
    """Log debug information with standard format"""
    default_logger.debug(message, context, component)

def log_info(message: str, context: Optional[dict] = None, component: Optional[str] = None):
    """Log an info message with standard format"""
    default_logger.info(message, context, component)

# Configure the default logger based on environment
env = os.getenv("ENVIRONMENT", "development")
config = get_logging_config(env)
default_logger = configure_logging(env)

# Export the configured logger
logger = default_logger