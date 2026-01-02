# Logging Utilities

This project includes comprehensive logging utilities for both frontend (TypeScript/JavaScript) and backend (Python) components.

## Frontend Logging (TypeScript)

The frontend logging utility is located at `src/lib/logger.ts` and provides structured logging capabilities.

### Usage

```typescript
import logger from './lib/logger';

// Basic logging
logger.info('Application started');
logger.debug('Debug information', { userId: 123 });
logger.warn('Warning message', { context: 'user-session' });
logger.error('Error occurred', { error: 'network timeout' });

// Logging with component context
logger.info('User clicked button', { buttonId: 'submit-btn' }, 'UserProfileComponent');

// Change log level
logger.setLevel(LogLevel.DEBUG);
logger.enable();  // Enable logging
logger.disable(); // Disable logging

// Set user ID for all logs
logger.setUserId('user-12345');

// Retrieve logs programmatically
const logs = logger.getLogs();
const errorLogs = logger.getLogsByLevel(LogLevel.ERROR);
const componentLogs = logger.getLogsByComponent('UserProfileComponent');
logger.clearLogs(); // Clear log history

// Export logs
const jsonLogs = logger.exportLogs('json');
const textLogs = logger.exportLogs('text');

// Download logs to file
logger.downloadLogs('my-logs.json');

// Enable remote logging
logger.enableRemoteLogging('https://example.com/api/logs');
logger.disableRemoteLogging();
```

### Features

- Multiple log levels: DEBUG, INFO, WARN, ERROR
- Structured logging with context and component tracking
- Timestamps for each log entry
- Configurable output format (JSON or text)
- Log history for debugging with filtering capabilities
- Console output with appropriate log levels
- File download capability for logs
- Remote logging to external services
- User ID tracking across all logs
- Log export functionality (JSON/text formats)

## Backend Logging (Python)

The backend logging utility is located at `backend/log_utils.py` and provides structured logging for Python components.

### Usage

```python
from backend.log_utils import logger, info, debug, warning, error, exception

# Using the logger instance
logger.info("Application started")
logger.debug("Debug information", {"user_id": 123}, "main_module")
logger.warning("Warning message", {"module": "auth"}, "auth_service")
logger.error("Error occurred", {"error_code": 500}, "api_handler")

# Using convenience functions
info("Application started")
debug("Debug information", {"user_id": 123}, "data_processor")
warning("Warning message", {"module": "auth"}, "auth_service")
error("Error occurred", {"error_code": 500}, "api_handler")

# Exception logging with traceback
try:
    raise ValueError("Test exception")
except ValueError:
    exception("Exception caught", {"operation": "data_processing"}, "data_processor")

# Retrieve logs programmatically
logs = logger.get_logs()
error_logs = logger.get_logs_by_level('ERROR')
component_logs = logger.get_logs_by_component('api_handler')
logger.clear_logs()  # Clear log history

# Export logs
json_logs = logger.export_logs('json')
text_logs = logger.export_logs('text')

# Save logs to file
logger.save_logs_to_file('logs/app_export.json', 'json')

# Initialize logger with file output and rotation
from backend.log_utils import StructuredLogger
file_logger = StructuredLogger(
    name="my_app",
    log_file="logs/app.log",
    max_bytes=10 * 1024 * 1024,  # 10MB
    backup_count=5,
    include_console=True
)
```

### Features

- Multiple log levels: DEBUG, INFO, WARNING, ERROR
- Structured logging with context and component tracking
- JSON-formatted context data
- Exception logging with tracebacks
- File logging with rotation support
- Standard Python logging integration
- Log history with filtering capabilities
- Log export functionality (JSON/text formats)
- Remote logging to external services
- Configurable log file rotation

## Configuration

Both logging utilities can be configured with various options:

### TypeScript Logger Options

```typescript
const customLogger = new Logger({
  level: LogLevel.DEBUG,           // Minimum log level to output
  enabled: true,                   // Whether logging is enabled
  format: 'json',                  // Output format: 'json' or 'text'
  logToConsole: true,              // Output to browser console
  logToFile: false,                // Trigger file download for each log (browser)
  logToRemote: false,              // Send logs to remote service
  remoteEndpoint: 'https://example.com/api/logs', // Remote logging endpoint
  userId: 'user-12345',            // User ID to include in all logs
  maxHistorySize: 1000             // Maximum number of logs to keep in memory
});
```

### Python Logger Options

```python
from backend.log_utils import StructuredLogger

logger = StructuredLogger(
    name="my_app",
    level=logging.INFO,
    log_file="logs/app.log",        # Path to log file
    max_bytes=10 * 1024 * 1024,     # Max file size before rotation (10MB)
    backup_count=5,                 # Number of backup files to keep
    include_console=True,            # Also log to console
    remote_endpoint="https://example.com/api/logs"  # Remote logging endpoint
)
```

## Best Practices

1. Always include relevant context in log messages
2. Use appropriate log levels (DEBUG for detailed info, INFO for general flow, WARN for potential issues, ERROR for problems)
3. Include component names when logging from specific components/modules
4. Avoid logging sensitive information like passwords or tokens
5. Use structured logging with consistent context keys
6. Regularly monitor logs for errors and warnings
7. Implement log rotation for file-based logging to manage disk space
8. Use remote logging for centralized log management in production
9. Set appropriate log levels for different environments (DEBUG for development, INFO/WARN/ERROR for production)