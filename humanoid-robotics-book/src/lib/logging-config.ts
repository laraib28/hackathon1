/**
 * Centralized logging configuration for the Humanoid Robotics Book project
 * Sets up default logging options and provides configuration presets
 */

import logger, { LoggerOptions } from './logger';

// Define configuration presets for different environments
const LOGGING_CONFIGS = {
  development: {
    level: 'DEBUG',
    logToConsole: true,
    logToFile: false,
    logToRemote: false,
    remoteEndpoint: undefined as string | undefined, // Explicitly set type
    maxHistorySize: 1000,
  },
  production: {
    level: 'INFO',
    logToConsole: true, // In browser, you might want to set this to false in production
    logToFile: false,
    logToRemote: true, // Enable remote logging in production
    remoteEndpoint: process.env.LOGGING_ENDPOINT || 'https://example.com/api/logs',
    maxHistorySize: 500,
  },
  testing: {
    level: 'DEBUG',
    logToConsole: true,
    logToFile: false,
    logToRemote: false,
    remoteEndpoint: undefined as string | undefined, // Explicitly set type
    maxHistorySize: 100,
  }
};

// Initialize logging based on environment
const env = process.env.NODE_ENV || 'development';
const config = LOGGING_CONFIGS[env as keyof typeof LOGGING_CONFIGS] || LOGGING_CONFIGS.development;

// Apply configuration
logger.setLevel(config.level as any);
if (config.logToRemote && config.remoteEndpoint) {
  logger.enableRemoteLogging(config.remoteEndpoint);
}

export interface LoggingConfig {
  userId?: string;
  component?: string;
  customEndpoint?: string;
}

/**
 * Configure logging with custom options
 */
export function configureLogging(options: LoggingConfig = {}): void {
  if (options.userId) {
    logger.setUserId(options.userId);
  }
  
  if (options.customEndpoint) {
    logger.enableRemoteLogging(options.customEndpoint);
  }
}

/**
 * Get the current logger instance
 */
export function getLogger() {
  return logger;
}

/**
 * Log an event with standard format
 */
export function logEvent(event: string, context?: Record<string, any>, component?: string): void {
  logger.info(`Event: ${event}`, context, component);
}

/**
 * Log an error with standard format
 */
export function logError(error: string | Error, context?: Record<string, any>, component?: string): void {
  if (typeof error === 'string') {
    logger.error(error, context, undefined, component);
  } else {
    logger.error(error.message, context, error, component);
  }
}

/**
 * Log a warning with standard format
 */
export function logWarning(warning: string, context?: Record<string, any>, component?: string): void {
  logger.warn(warning, context, component);
}

/**
 * Log debug information with standard format
 */
export function logDebug(message: string, context?: Record<string, any>, component?: string): void {
  logger.debug(message, context, component);
}

/**
 * Log an info message with standard format
 */
export function logInfo(message: string, context?: Record<string, any>, component?: string): void {
  logger.info(message, context, component);
}

// Export the logger as default
export default logger;