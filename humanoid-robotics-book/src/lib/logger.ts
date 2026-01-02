/**
 * Logging utility for the Humanoid Robotics Book project
 * Provides structured logging functionality for both client and server environments
 */

export interface LogEntry {
  timestamp: Date;
  level: LogLevel;
  message: string;
  context?: Record<string, any>;
  stack?: string;
  component?: string; // Additional field for component tracking
  userId?: string; // Additional field for user tracking if needed
}

export enum LogLevel {
  DEBUG = 'DEBUG',
  INFO = 'INFO',
  WARN = 'WARN',
  ERROR = 'ERROR',
}

export interface LoggerOptions {
  level?: LogLevel;
  enabled?: boolean;
  format?: 'json' | 'text';
  destination?: 'console' | 'file' | 'both';
  maxHistorySize?: number; // Maximum number of logs to keep in memory
  logToConsole?: boolean; // Whether to log to console
  logToFile?: boolean; // Whether to log to file (in browser, this means download)
  logToRemote?: boolean; // Whether to send logs to remote service
  remoteEndpoint?: string; // Endpoint to send logs to
  userId?: string; // User ID to include in logs
}

class Logger {
  private level: LogLevel;
  private enabled: boolean;
  private format: 'json' | 'text';
  private logToConsole: boolean;
  private logToFile: boolean;
  private logToRemote: boolean;
  private remoteEndpoint?: string;
  private userId?: string;
  private maxHistorySize: number;
  private logHistory: LogEntry[] = [];

  constructor(options: LoggerOptions = {}) {
    this.level = options.level || LogLevel.INFO;
    this.enabled = options.enabled !== undefined ? options.enabled : true;
    this.format = options.format || 'text';
    this.logToConsole = options.logToConsole !== undefined ? options.logToConsole : true;
    this.logToFile = options.logToFile !== undefined ? options.logToFile : false;
    this.logToRemote = options.logToRemote !== undefined ? options.logToRemote : false;
    this.remoteEndpoint = options.remoteEndpoint;
    this.userId = options.userId;
    this.maxHistorySize = options.maxHistorySize || 1000;
  }

  private shouldLog(level: LogLevel): boolean {
    const levels = [LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR];
    return this.enabled && levels.indexOf(level) >= levels.indexOf(this.level);
  }

  private async sendLogToRemote(entry: LogEntry): Promise<void> {
    if (!this.logToRemote || !this.remoteEndpoint) return;

    try {
      // In a real implementation, this would send to a logging service
      // For now, we'll just log to console to simulate
      console.log('Sending log to remote:', this.remoteEndpoint, entry);

      // Uncomment the following code when you have a backend logging endpoint
      /*
      await fetch(this.remoteEndpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          ...entry,
          timestamp: entry.timestamp.toISOString(),
        }),
      });
      */
    } catch (error) {
      console.error('Failed to send log to remote service:', error);
    }
  }

  private writeLog(entry: LogEntry): void {
    if (!this.shouldLog(entry.level)) return;

    // Add user ID if available
    if (this.userId && !entry.userId) {
      entry.userId = this.userId;
    }

    let output: string;

    if (this.format === 'json') {
      output = JSON.stringify({
        ...entry,
        timestamp: entry.timestamp.toISOString(),
      });
    } else {
      const timestamp = entry.timestamp.toISOString();
      const level = entry.level.padEnd(5);
      const message = entry.message;
      const component = entry.component ? ` [${entry.component}]` : '';
      const context = entry.context ? ` ${JSON.stringify(entry.context)}` : '';
      output = `${timestamp} [${level}]${component} ${message}${context}`;

      if (entry.stack) {
        output += `\n${entry.stack}`;
      }
    }

    // Add to log history for retrieval
    this.logHistory.push(entry);

    // Limit history to prevent memory issues
    if (this.logHistory.length > this.maxHistorySize) {
      this.logHistory = this.logHistory.slice(-Math.floor(this.maxHistorySize / 2)); // Keep half when exceeding
    }

    // Output to console
    if (this.logToConsole) {
      switch (entry.level) {
        case LogLevel.ERROR:
          console.error(output);
          break;
        case LogLevel.WARN:
          console.warn(output);
          break;
        case LogLevel.DEBUG:
          console.debug(output);
          break;
        default:
          console.log(output);
      }
    }

    // Send to remote service if configured
    if (this.logToRemote) {
      this.sendLogToRemote(entry);
    }

    // Log to file if configured (in browser, this would trigger a download)
    if (this.logToFile) {
      this.saveLogToFile(output);
    }
  }

  private saveLogToFile(content: string): void {
    // In browser environment, we can create a downloadable file
    if (typeof window !== 'undefined' && window.Blob && window.URL) {
      try {
        const blob = new Blob([content + '\n'], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `log-${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.txt`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
      } catch (error) {
        console.error('Failed to save log to file:', error);
      }
    }
  }

  debug(message: string, context?: Record<string, any>, component?: string): void {
    this.writeLog({
      timestamp: new Date(),
      level: LogLevel.DEBUG,
      message,
      context,
      component,
    });
  }

  info(message: string, context?: Record<string, any>, component?: string): void {
    this.writeLog({
      timestamp: new Date(),
      level: LogLevel.INFO,
      message,
      context,
      component,
    });
  }

  warn(message: string, context?: Record<string, any>, component?: string): void {
    this.writeLog({
      timestamp: new Date(),
      level: LogLevel.WARN,
      message,
      context,
      component,
    });
  }

  error(message: string, context?: Record<string, any>, error?: Error, component?: string): void {
    this.writeLog({
      timestamp: new Date(),
      level: LogLevel.ERROR,
      message,
      context,
      stack: error?.stack,
      component,
    });
  }

  getLogs(): LogEntry[] {
    return [...this.logHistory]; // Return a copy
  }

  getLogsByLevel(level: LogLevel): LogEntry[] {
    return this.logHistory.filter(entry => entry.level === level);
  }

  getLogsByComponent(component: string): LogEntry[] {
    return this.logHistory.filter(entry => entry.component === component);
  }

  clearLogs(): void {
    this.logHistory = [];
  }

  setLevel(level: LogLevel): void {
    this.level = level;
  }

  setUserId(userId: string): void {
    this.userId = userId;
  }

  enable(): void {
    this.enabled = true;
  }

  disable(): void {
    this.enabled = false;
  }

  enableRemoteLogging(endpoint: string): void {
    this.logToRemote = true;
    this.remoteEndpoint = endpoint;
  }

  disableRemoteLogging(): void {
    this.logToRemote = false;
    this.remoteEndpoint = undefined;
  }

  exportLogs(format: 'json' | 'text' = 'json'): string {
    if (format === 'json') {
      return JSON.stringify(this.logHistory.map(entry => ({
        ...entry,
        timestamp: entry.timestamp.toISOString(),
      })), null, 2);
    } else {
      return this.logHistory.map(entry => {
        const timestamp = entry.timestamp.toISOString();
        const level = entry.level.padEnd(5);
        const message = entry.message;
        const component = entry.component ? ` [${entry.component}]` : '';
        const context = entry.context ? ` ${JSON.stringify(entry.context)}` : '';
        let output = `${timestamp} [${level}]${component} ${message}${context}`;

        if (entry.stack) {
          output += `\n${entry.stack}`;
        }
        return output;
      }).join('\n');
    }
  }

  downloadLogs(filename: string = `logs-${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.txt`): void {
    const content = this.exportLogs();
    if (typeof window !== 'undefined' && window.Blob && window.URL) {
      try {
        const blob = new Blob([content], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
      } catch (error) {
        console.error('Failed to download logs:', error);
      }
    }
  }
}

// Create a global logger instance
const logger = new Logger();

// Export the logger instance and types
export { logger };
export default logger;