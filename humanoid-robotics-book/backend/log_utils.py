"""
Logging utility for the Humanoid Robotics Book backend
Provides structured logging functionality for backend services
"""

import logging
import json
import os
from datetime import datetime
from typing import Dict, Any, Optional, List
from pathlib import Path
import sys


class StructuredLogger:
    def __init__(
        self,
        name: str = __name__,
        level: int = logging.INFO,
        log_file: Optional[str] = None,
        max_bytes: int = 10 * 1024 * 1024,  # 10MB
        backup_count: int = 5,
        include_console: bool = True,
        remote_endpoint: Optional[str] = None
    ):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        # Prevent adding multiple handlers if logger already has handlers
        if not self.logger.handlers:
            # Console handler
            if include_console:
                console_handler = logging.StreamHandler(sys.stdout)
                console_formatter = logging.Formatter(
                    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
                )
                console_handler.setFormatter(console_formatter)
                self.logger.addHandler(console_handler)

            # File handler with rotation
            if log_file:
                from logging.handlers import RotatingFileHandler
                file_handler = RotatingFileHandler(
                    log_file,
                    maxBytes=max_bytes,
                    backupCount=backup_count
                )
                file_formatter = logging.Formatter(
                    '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
                )
                file_handler.setFormatter(file_formatter)
                self.logger.addHandler(file_handler)

        self.remote_endpoint = remote_endpoint
        self._log_history: List[Dict[str, Any]] = []
        self.max_history_size = 1000

    def _add_to_history(self, level: int, message: str, context: Optional[Dict[str, Any]] = None):
        """Add log entry to in-memory history for debugging purposes"""
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "level": logging.getLevelName(level),
            "message": message,
            "context": context or {}
        }

        self._log_history.append(log_entry)

        # Keep history size manageable
        if len(self._log_history) > self.max_history_size:
            self._log_history = self._log_history[-500:]  # Keep last 500 entries

    def _log(self, level: int, message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
        """Internal logging method that handles context and remote logging"""
        # Prepare the log entry for history
        log_data = {
            "timestamp": datetime.now().isoformat(),
            "level": logging.getLevelName(level),
            "message": message,
            "context": context or {},
            "component": component
        }

        # Add to history
        self._add_to_history(level, message, context)

        # Format the log message with context
        if context:
            log_entry = f"{message} | Component: {component or 'unknown'} | Context: {json.dumps(context)}"
        else:
            log_entry = f"{message} | Component: {component or 'unknown'}"

        # Log to standard handlers
        self.logger.log(level, log_entry)

        # Send to remote endpoint if configured
        if self.remote_endpoint:
            self._send_to_remote(log_data)

    def _send_to_remote(self, log_data: Dict[str, Any]):
        """Send log data to remote endpoint"""
        try:
            # In a real implementation, this would send to a remote logging service
            # For now, we'll just print to simulate
            print(f"Sending log to remote: {self.remote_endpoint}, data: {log_data}")

            # Uncomment the following code when you have an HTTP client available
            # import requests
            # requests.post(self.remote_endpoint, json=log_data)
        except Exception as e:
            self.logger.error(f"Failed to send log to remote endpoint: {e}")

    def debug(self, message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
        self._log(logging.DEBUG, message, context, component)

    def info(self, message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
        self._log(logging.INFO, message, context, component)

    def warning(self, message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
        self._log(logging.WARNING, message, context, component)

    def error(self, message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
        self._log(logging.ERROR, message, context, component)

    def exception(self, message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
        """Log an exception with traceback"""
        if context:
            log_entry = f"{message} | Component: {component or 'unknown'} | Context: {json.dumps(context)}"
        else:
            log_entry = f"{message} | Component: {component or 'unknown'}"

        self.logger.exception(log_entry)

        # Also add to history
        self._add_to_history(logging.ERROR, message, context)

    def get_logs(self) -> List[Dict[str, Any]]:
        """Get all logs from history"""
        return self._log_history.copy()

    def get_logs_by_level(self, level: str) -> List[Dict[str, Any]]:
        """Get logs filtered by level"""
        return [log for log in self._log_history if log["level"] == level.upper()]

    def get_logs_by_component(self, component: str) -> List[Dict[str, Any]]:
        """Get logs filtered by component"""
        return [log for log in self._log_history if log.get("component") == component]

    def clear_logs(self):
        """Clear the log history"""
        self._log_history = []

    def export_logs(self, format: str = 'json') -> str:
        """Export logs in specified format"""
        if format.lower() == 'json':
            return json.dumps(self._log_history, indent=2)
        else:  # text format
            lines = []
            for log in self._log_history:
                timestamp = log["timestamp"]
                level = log["level"].ljust(5)
                message = log["message"]
                component = log.get("component", "unknown")
                context = json.dumps(log.get("context", {}))
                lines.append(f"{timestamp} [{level}] [{component}] {message} | Context: {context}")
            return "\n".join(lines)

    def save_logs_to_file(self, filepath: str, format: str = 'json'):
        """Save logs to a file"""
        content = self.export_logs(format)
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(content)


# Create a global logger instance with default settings
log_file_path = os.getenv("LOG_FILE", "logs/app.log")
os.makedirs(os.path.dirname(log_file_path), exist_ok=True)  # Create logs directory if it doesn't exist

logger = StructuredLogger(
    name="humanoid_robotics_book",
    log_file=log_file_path,
    include_console=True
)

# Convenience functions that use the global logger
def debug(message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
    logger.debug(message, context, component)

def info(message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
    logger.info(message, context, component)

def warning(message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
    logger.warning(message, context, component)

def error(message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
    logger.error(message, context, component)

def exception(message: str, context: Optional[Dict[str, Any]] = None, component: Optional[str] = None):
    logger.exception(message, context, component)


# Example usage
if __name__ == "__main__":
    # Test the logger
    info("Application started")
    debug("Debug information", {"user_id": 123, "session": "abc"}, "main")
    warning("This is a warning", {"module": "auth"}, "auth_service")
    error("An error occurred", {"error_code": 500, "endpoint": "/api/users"}, "api_handler")

    try:
        raise ValueError("Test exception")
    except ValueError:
        exception("Exception caught", {"operation": "data_processing"}, "data_processor")

    # Test log retrieval
    print("\nRetrieving logs:")
    logs = logger.get_logs()
    print(f"Total logs: {len(logs)}")

    # Export logs
    print("\nExporting logs to file:")
    logger.save_logs_to_file("logs/test_export.json", "json")
    print("Logs exported to logs/test_export.json")