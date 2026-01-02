"""
Example usage of the logging utilities in Python backend components
"""

from backend.log_utils import logger, debug, info, warning, error, exception


def example_usage():
    """Demonstrate various ways to use the logging utilities"""

    # Method 1: Using the logger instance directly with component context
    logger.info("Starting example usage function", {"function": "example_usage"}, "Main")
    logger.debug("Debug information", {"function": "example_usage", "step": 1}, "Main")
    logger.warning("This is a sample warning", {"module": "example", "severity": "low"}, "Main")

    # Method 2: Using convenience functions with component context
    info("Informational message using convenience function", component="Main")
    debug("Debug message with context", {"context": "convenience_functions", "value": 42}, "Utils")

    # Method 3: Error logging with component context
    try:
        # Simulate an error condition
        result = 10 / 0
    except ZeroDivisionError as e:
        logger.exception("Division by zero error occurred", {"operation": "division", "numerator": 10, "denominator": 0}, "Calculator")

        # Alternative way to log exceptions
        error("Handled division by zero", {"error_type": "ZeroDivisionError", "handled": True}, "ErrorHandler")

    # Method 4: Different log levels with component context
    info("Processing user request", {"user_id": 12345, "request_type": "data_fetch"}, "RequestProcessor")
    debug("Detailed processing info", {
        "algorithm": "search_v2",
        "parameters": {"limit": 10, "sort": "date"},
        "cache_hit": False
    }, "DataProcessor")
    warning("High memory usage detected", {"memory_mb": 850, "threshold_mb": 800}, "SystemMonitor")
    error("Database connection failed", {
        "host": "db.example.com",
        "port": 5432,
        "retry_count": 3
    }, "DatabaseConnection")

    # Method 5: Log retrieval and filtering
    print("\n--- Log Retrieval Examples ---")
    all_logs = logger.get_logs()
    print(f"Total logs: {len(all_logs)}")

    error_logs = logger.get_logs_by_level('ERROR')
    print(f"Error logs: {len(error_logs)}")

    processor_logs = logger.get_logs_by_component('RequestProcessor')
    print(f"RequestProcessor logs: {len(processor_logs)}")

    # Method 6: Export logs
    print("\n--- Exporting Logs ---")
    json_logs = logger.export_logs('json')
    print(f"JSON export length: {len(json_logs)} characters")

    text_logs = logger.export_logs('text')
    print(f"Text export length: {len(text_logs)} characters")

    # Save logs to file
    logger.save_logs_to_file('logs/backend_example_export.json', 'json')
    print("Logs saved to logs/backend_example_export.json")

    logger.info("Completed example usage function", component="Main")


if __name__ == "__main__":
    print("Running advanced logging example...")
    example_usage()
    print("Check console output and logs/backend_example_export.json for logged messages.")