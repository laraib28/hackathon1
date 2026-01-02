/**
 * Authentication logging service for the Humanoid Robotics Book project
 * Provides logging functionality specifically for authentication events
 */

import logger from '../lib/logger';
import { User } from '../services/authService';

export interface AuthLogContext {
  userId?: string;
  email?: string;
  ip?: string;
  userAgent?: string;
  sessionId?: string;
  error?: string;
  path?: string;
  method?: string;
  displayName?: string;
  action?: string;
  timestamp?: string;
}

export enum AuthEvent {
  LOGIN_SUCCESS = 'login_success',
  LOGIN_FAILURE = 'login_failure',
  SIGNUP_SUCCESS = 'signup_success',
  SIGNUP_FAILURE = 'signup_failure',
  LOGOUT = 'logout',
  PASSWORD_RESET_REQUEST = 'password_reset_request',
  PASSWORD_RESET_SUCCESS = 'password_reset_success',
  PASSWORD_RESET_FAILURE = 'password_reset_failure',
  SESSION_EXPIRED = 'session_expired',
  SESSION_RENEWED = 'session_renewed',
  PROFILE_UPDATE = 'profile_update',
  PROFILE_UPDATE_FAILURE = 'profile_update_failure',
  UNAUTHORIZED_ACCESS = 'unauthorized_access',
  TOKEN_REFRESH_SUCCESS = 'token_refresh_success',
  TOKEN_REFRESH_FAILURE = 'token_refresh_failure',
}

export class AuthLogger {
  private static instance: AuthLogger;
  
  private constructor() {}

  public static getInstance(): AuthLogger {
    if (!AuthLogger.instance) {
      AuthLogger.instance = new AuthLogger();
    }
    return AuthLogger.instance;
  }

  private getContext(user?: User, additionalContext?: Partial<AuthLogContext>): AuthLogContext {
    const context: AuthLogContext = {
      ...additionalContext,
      userId: user?.id,
      email: user?.email,
      userAgent: typeof window !== 'undefined' ? navigator.userAgent : undefined,
    };

    return context;
  }

  public logAuthEvent(
    event: AuthEvent,
    user?: User,
    additionalContext?: Partial<AuthLogContext>,
    component: string = 'AuthLogger'
  ): void {
    const context = this.getContext(user, additionalContext);

    switch (event) {
      case AuthEvent.LOGIN_SUCCESS:
        logger.info(`User login successful`, context, component);
        break;
      case AuthEvent.LOGIN_FAILURE:
        logger.warn(`User login failed`, context, component);
        break;
      case AuthEvent.SIGNUP_SUCCESS:
        logger.info(`User signup successful`, context, component);
        break;
      case AuthEvent.SIGNUP_FAILURE:
        logger.error(`User signup failed`, context, undefined, component);
        break;
      case AuthEvent.LOGOUT:
        logger.info(`User logged out`, context, component);
        break;
      case AuthEvent.PASSWORD_RESET_REQUEST:
        logger.info(`Password reset requested`, context, component);
        break;
      case AuthEvent.PASSWORD_RESET_SUCCESS:
        logger.info(`Password reset successful`, context, component);
        break;
      case AuthEvent.PASSWORD_RESET_FAILURE:
        logger.error(`Password reset failed`, context, undefined, component);
        break;
      case AuthEvent.SESSION_EXPIRED:
        logger.warn(`Session expired`, context, component);
        break;
      case AuthEvent.SESSION_RENEWED:
        logger.info(`Session renewed`, context, component);
        break;
      case AuthEvent.PROFILE_UPDATE:
        logger.info(`Profile updated`, context, component);
        break;
      case AuthEvent.PROFILE_UPDATE_FAILURE:
        logger.error(`Profile update failed`, context, undefined, component);
        break;
      case AuthEvent.UNAUTHORIZED_ACCESS:
        logger.warn(`Unauthorized access attempt`, context, component);
        break;
      case AuthEvent.TOKEN_REFRESH_SUCCESS:
        logger.debug(`Token refresh successful`, context, component);
        break;
      case AuthEvent.TOKEN_REFRESH_FAILURE:
        logger.warn(`Token refresh failed`, context, component);
        break;
      default:
        logger.info(`Auth event: ${event}`, context, component);
    }
  }

  public logLoginSuccess(user: User, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.LOGIN_SUCCESS, user, additionalContext);
  }

  public logLoginFailure(email: string, error: string, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.LOGIN_FAILURE, undefined, {
      ...additionalContext,
      email,
      error
    });
  }

  public logSignupSuccess(user: User, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.SIGNUP_SUCCESS, user, additionalContext);
  }

  public logSignupFailure(email: string, error: string, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.SIGNUP_FAILURE, undefined, {
      ...additionalContext,
      email,
      error
    });
  }

  public logLogout(user?: User, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.LOGOUT, user, additionalContext);
  }

  public logUnauthorizedAccess(path: string, user?: User, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.UNAUTHORIZED_ACCESS, user, {
      ...additionalContext,
      path
    });
  }

  public logPasswordResetRequest(email: string, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.PASSWORD_RESET_REQUEST, undefined, {
      ...additionalContext,
      email
    });
  }

  public logSessionExpired(user?: User, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.SESSION_EXPIRED, user, additionalContext);
  }

  public logSessionRenewed(user?: User, additionalContext?: Partial<AuthLogContext>): void {
    this.logAuthEvent(AuthEvent.SESSION_RENEWED, user, additionalContext);
  }
}

// Create and export a singleton instance
const authLogger = AuthLogger.getInstance();
export default authLogger;