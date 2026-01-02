/**
 * Example logging component for the Humanoid Robotics Book
 * Demonstrates how to use the logging utilities in the application
 */

import React, { useEffect } from 'react';
import logger, { LogLevel } from '../lib/logger';

const LoggingExample: React.FC = () => {
  useEffect(() => {
    // Log when this component mounts
    logger.info('LoggingExample component mounted', {}, 'LoggingExample');

    // Example of different log levels with component context
    logger.debug('Debug information for LoggingExample component', {
      component: 'LoggingExample',
      timestamp: new Date().toISOString(),
    }, 'LoggingExample');

    logger.info('User viewed logging documentation', {
      page: 'logging-example',
      userAgent: navigator.userAgent,
    }, 'LoggingExample');

    // Example of error logging
    try {
      // Simulate a potential issue
      throw new Error('Simulated error for demonstration');
    } catch (error) {
      logger.error(
        'Error occurred in LoggingExample component',
        {
          component: 'LoggingExample',
          error: error instanceof Error ? error.message : 'Unknown error'
        },
        error instanceof Error ? error : undefined,
        'LoggingExample'
      );
    }

    // Set a user ID for all subsequent logs
    logger.setUserId('user-12345');

    // Clean up logging when component unmounts
    return () => {
      logger.info('LoggingExample component unmounted', {}, 'LoggingExample');
    };
  }, []);

  const handleDebugLog = () => {
    logger.debug('Debug button clicked', {
      button: 'debug-log',
      timestamp: new Date().toISOString(),
    }, 'LoggingExample');
  };

  const handleInfoLog = () => {
    logger.info('Info button clicked', {
      button: 'info-log',
      timestamp: new Date().toISOString(),
    }, 'LoggingExample');
  };

  const handleErrorLog = () => {
    logger.error('Error button clicked intentionally', {
      button: 'error-log',
      intentional: true,
    }, undefined, 'LoggingExample');
  };

  const handleGetLogs = () => {
    const logs = logger.getLogs();
    logger.info(`Retrieved ${logs.length} log entries`, {
      action: 'get-logs',
      timestamp: new Date().toISOString(),
    }, 'LoggingExample');
    console.table(logs.slice(-10)); // Show last 10 logs in table format
  };

  const handleGetErrorLogs = () => {
    const errorLogs = logger.getLogsByLevel(LogLevel.ERROR);
    logger.info(`Retrieved ${errorLogs.length} error log entries`, {
      action: 'get-error-logs',
      timestamp: new Date().toISOString(),
    }, 'LoggingExample');
    console.table(errorLogs);
  };

  const handleGetComponentLogs = () => {
    const componentLogs = logger.getLogsByComponent('LoggingExample');
    logger.info(`Retrieved ${componentLogs.length} component log entries`, {
      action: 'get-component-logs',
      timestamp: new Date().toISOString(),
    }, 'LoggingExample');
    console.table(componentLogs);
  };

  const handleExportLogs = () => {
    const exportedLogs = logger.exportLogs('json');
    logger.info('Logs exported to JSON format', {
      action: 'export-logs',
      timestamp: new Date().toISOString(),
      logCount: logger.getLogs().length,
    }, 'LoggingExample');
    console.log(exportedLogs);
  };

  const handleDownloadLogs = () => {
    logger.downloadLogs(`humanoid-robotics-logs-${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.json`);
    logger.info('Logs download initiated', {
      action: 'download-logs',
      timestamp: new Date().toISOString(),
    }, 'LoggingExample');
  };

  const handleEnableRemoteLogging = () => {
    logger.enableRemoteLogging('https://example.com/api/logs');
    logger.info('Remote logging enabled', {
      action: 'enable-remote-logging',
      endpoint: 'https://example.com/api/logs',
    }, 'LoggingExample');
  };

  const handleDisableRemoteLogging = () => {
    logger.disableRemoteLogging();
    logger.info('Remote logging disabled', {
      action: 'disable-remote-logging',
    }, 'LoggingExample');
  };

  return (
    <div className="logging-example">
      <h2>Advanced Logging Utility Demo</h2>
      <p>This component demonstrates the comprehensive logging utilities available in the Humanoid Robotics Book project.</p>

      <div style={{ margin: '20px 0' }}>
        <button onClick={handleDebugLog} style={{ margin: '5px', padding: '10px' }}>
          Log Debug Message
        </button>
        <button onClick={handleInfoLog} style={{ margin: '5px', padding: '10px' }}>
          Log Info Message
        </button>
        <button onClick={handleErrorLog} style={{ margin: '5px', padding: '10px' }}>
          Log Error Message
        </button>
        <button onClick={handleGetLogs} style={{ margin: '5px', padding: '10px' }}>
          View All Logs
        </button>
        <button onClick={handleGetErrorLogs} style={{ margin: '5px', padding: '10px' }}>
          View Error Logs
        </button>
        <button onClick={handleGetComponentLogs} style={{ margin: '5px', padding: '10px' }}>
          View Component Logs
        </button>
        <button onClick={handleExportLogs} style={{ margin: '5px', padding: '10px' }}>
          Export Logs (Console)
        </button>
        <button onClick={handleDownloadLogs} style={{ margin: '5px', padding: '10px' }}>
          Download Logs
        </button>
        <button onClick={handleEnableRemoteLogging} style={{ margin: '5px', padding: '10px' }}>
          Enable Remote Logging
        </button>
        <button onClick={handleDisableRemoteLogging} style={{ margin: '5px', padding: '10px' }}>
          Disable Remote Logging
        </button>
      </div>

      <div style={{ marginTop: '20px', padding: '15px', backgroundColor: '#f5f5f5', borderRadius: '5px' }}>
        <h3>How to Use Advanced Logging:</h3>
        <pre style={{ whiteSpace: 'pre-wrap', overflowX: 'auto' }}>
{`import logger from '../lib/logger';

// Basic logging with component context
logger.info('Application started', { userId: 123 }, 'MainComponent');

// Set user ID for all logs
logger.setUserId('user-12345');

// Retrieve logs programmatically
const logs = logger.getLogs();
const errorLogs = logger.getLogsByLevel(LogLevel.ERROR);
const componentLogs = logger.getLogsByComponent('UserProfileComponent');

// Export logs
const jsonLogs = logger.exportLogs('json');
const textLogs = logger.exportLogs('text');

// Download logs to file
logger.downloadLogs('my-logs.json');

// Enable remote logging
logger.enableRemoteLogging('https://example.com/api/logs');
logger.disableRemoteLogging();`}
        </pre>
      </div>
    </div>
  );
};

export default LoggingExample;