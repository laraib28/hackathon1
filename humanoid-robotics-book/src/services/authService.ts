/**
 * Authentication Service - Better Auth Integration
 * Centralized auth methods using Better Auth
 */
import { authClient, signIn as betterAuthSignIn, signUp as betterAuthSignUp, signOut as betterAuthSignOut, useSession } from '../lib/auth-client-safe';
import authLogger from './auth-logger';

export interface User {
  id: string;
  email: string;
  name: string;
  emailVerified: boolean;
  image?: string | null;
  createdAt: Date;
}

/**
 * Sign up with email and password
 */
export async function signup(email: string, password: string, name?: string): Promise<void> {
  try {
    const result = await betterAuthSignUp.email({
      email,
      password,
      name: name || email.split('@')[0] || 'Anonymous',
    });

    if (result.error) {
      authLogger.logSignupFailure(email, result.error.message || 'Signup failed');
      throw new Error(result.error.message || 'Signup failed');
    }

    // Get session after successful signup
    const sessionResult = await authClient.getSession();

    // Log successful signup with user data
    if (sessionResult.data?.user) {
      const user: User = {
        id: sessionResult.data.user.id,
        email: sessionResult.data.user.email,
        name: sessionResult.data.user.name || sessionResult.data.user.email.split('@')[0] || 'Anonymous',
        emailVerified: sessionResult.data.user.emailVerified || false,
        image: sessionResult.data.user.image,
        createdAt: sessionResult.data.user.createdAt || new Date(),
      };
      authLogger.logSignupSuccess(user);
    }
  } catch (error) {
    console.error('Signup error:', error);
    // Error is already logged by authLogger in case of failure
    throw error;
  }
}

/**
 * Sign in with email and password
 */
export async function login(email: string, password: string): Promise<void> {
  try {
    console.log('🔐 Login attempt:', { email });
    console.log('📡 Auth client config:', authClient);

    const result = await betterAuthSignIn.email({
      email,
      password,
    });

    console.log('📥 Login result:', result);

    if (result.error) {
      console.error('❌ Login error from Better Auth:', result.error);
      authLogger.logLoginFailure(email, result.error.message || 'Login failed');
      throw new Error(result.error.message || 'Login failed');
    }

    // Get session after successful login
    const sessionResult = await authClient.getSession();
    console.log('📋 Session result:', sessionResult);

    // Log successful login with user data
    if (sessionResult.data?.user) {
      const user: User = {
        id: sessionResult.data.user.id,
        email: sessionResult.data.user.email,
        name: sessionResult.data.user.name || sessionResult.data.user.email.split('@')[0] || 'Anonymous',
        emailVerified: sessionResult.data.user.emailVerified || false,
        image: sessionResult.data.user.image,
        createdAt: sessionResult.data.user.createdAt || new Date(),
      };
      authLogger.logLoginSuccess(user);

      // Store auth token in localStorage for chatbot requests
      if (sessionResult.data?.session?.token) {
        localStorage.setItem('auth_token', sessionResult.data.session.token);
      }
    }
  } catch (error) {
    console.error('🚨 Login error caught:', error);
    console.error('🔍 Error type:', error?.constructor?.name);
    console.error('🔍 Error message:', error?.message);
    console.error('🔍 Full error:', JSON.stringify(error, null, 2));
    // Error is already logged by authLogger in case of failure
    throw error;
  }
}

/**
 * Sign out current user
 */
export async function logout(): Promise<void> {
  try {
    // Get current user before logout for logging
    const { data: session } = await authClient.getSession();
    const user = session?.user ? {
      id: session.user.id,
      email: session.user.email,
      name: session.user.name || session.user.email.split('@')[0] || 'Anonymous',
      emailVerified: session.user.emailVerified || false,
      image: session.user.image,
      createdAt: session.user.createdAt || new Date(),
    } : null;

    await betterAuthSignOut();

    // Clear auth token from localStorage
    localStorage.removeItem('auth_token');

    // Session data is cleared by the signOut function

    // Log logout event
    if (user) {
      authLogger.logLogout(user);
    } else {
      authLogger.logLogout();
    }
  } catch (error) {
    console.error('Logout error:', error);
    // Still proceed even if logout fails
  }
}

/**
 * Sign in with Google
 */
export async function loginWithGoogle(): Promise<void> {
  try {
    const result = await betterAuthSignIn.social({
      provider: 'google',
    });

    if (result.error) {
      authLogger.logLoginFailure('', result.error.message || 'Google login failed');
      throw new Error(result.error.message || 'Google login failed');
    }

    // Get session after successful Google login
    const sessionResult = await authClient.getSession();

    // Log successful login with user data
    if (sessionResult.data?.user) {
      const user: User = {
        id: sessionResult.data.user.id,
        email: sessionResult.data.user.email,
        name: sessionResult.data.user.name || sessionResult.data.user.email.split('@')[0] || 'Anonymous',
        emailVerified: sessionResult.data.user.emailVerified || false,
        image: sessionResult.data.user.image,
        createdAt: sessionResult.data.user.createdAt || new Date(),
      };
      authLogger.logLoginSuccess(user);
    }
  } catch (error) {
    console.error('Google login error:', error);
    throw error;
  }
}

/**
 * Reset password (if supported by Better Auth config)
 */
export async function resetPassword(email: string): Promise<void> {
  // This would require Better Auth to be configured with password reset
  // For now, just throw an error or implement when needed
  console.warn('Password reset not yet implemented with Better Auth');
  authLogger.logPasswordResetRequest(email);
  throw new Error('Password reset not yet implemented');
}

/**
 * Update user profile
 */
export async function updateUserProfile(displayName?: string, photoURL?: string): Promise<void> {
  try {
    // Use the enhanced updateProfile function from auth-client
    await import('../lib/auth-client').then(async (module) => {
      await module.updateProfile(displayName, photoURL);
    });
  } catch (error) {
    console.error('Update profile error:', error);
    throw error;
  }
}

/**
 * Get current user from session
 */
export async function getCurrentUser(): Promise<User | null> {
  try {
    const { data: session } = await authClient.getSession();

    if (!session?.user) {
      return null;
    }

    return {
      id: session.user.id,
      email: session.user.email,
      name: session.user.name || session.user.email.split('@')[0] || 'Anonymous',
      emailVerified: session.user.emailVerified || false,
      image: session.user.image,
      createdAt: session.user.createdAt || new Date(),
    };
  } catch (error) {
    console.error('Get current user error:', error);
    return null;
  }
}

/**
 * Get user preferences
 */
export async function getUserPreferences(): Promise<{ language?: string; theme?: string; chatHistory?: boolean } | null> {
  try {
    const { getUserPreferences: getUserPrefs } = await import('../lib/auth-client');
    return await getUserPrefs();
  } catch (error) {
    console.error('Get user preferences error:', error);
    return { language: 'en', theme: 'system' }; // Default preferences
  }
}

/**
 * Update user preferences
 */
export async function updateUserPreferences(preferences: { language?: string; theme?: string; chatHistory?: boolean }): Promise<void> {
  try {
    const { updateUserPreferences: updatePrefs } = await import('../lib/auth-client');
    await updatePrefs(preferences);
  } catch (error) {
    console.error('Update user preferences error:', error);
    throw error;
  }
}

/**
 * Get auth token for API requests
 */
export async function getAuthToken(): Promise<string | null> {
  try {
    const { data: session } = await authClient.getSession();
    return session?.session?.token || null;
  } catch (error) {
    console.error('Get auth token error:', error);
    return null;
  }
}

/**
 * Check if user is authenticated
 */
export async function isAuthenticated(): Promise<boolean> {
  try {
    const { data: session } = await authClient.getSession();
    return !!session?.user;
  } catch (error) {
    console.error('Check auth status error:', error);
    return false;
  }
}

/**
 * Auth state change listener (for compatibility with old code)
 */
export type AuthStateChangeCallback = (user: User | null) => void;

export function onAuthStateChanged(callback: AuthStateChangeCallback): () => void {
  // Better Auth uses React hooks for session management
  // This is a compatibility layer - prefer using useSession hook in components
  let isSubscribed = true;

  // Check session immediately
  getCurrentUser().then((user) => {
    if (isSubscribed) {
      callback(user);
    }
  });

  // Return unsubscribe function
  return () => {
    isSubscribed = false;
  };
}

// Re-export useSession hook for React components
export { useSession };
