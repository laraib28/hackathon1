/**
 * Authentication Service - Better Auth Integration
 * Centralized auth methods using Better Auth
 */
import { authClient, signIn as betterAuthSignIn, signUp as betterAuthSignUp, signOut as betterAuthSignOut, useSession } from '../lib/auth-client';

export interface User {
  id: string;
  email: string;
  name: string;
  emailVerified: boolean;
  image?: string;
  createdAt: Date;
}

/**
 * Sign up with email and password
 */
export async function signup(email: string, password: string, name?: string): Promise<void> {
  const result = await betterAuthSignUp.email({
    email,
    password,
    name: name || email.split('@')[0],
  });

  if (result.error) {
    throw new Error(result.error.message || 'Signup failed');
  }
}

/**
 * Sign in with email and password
 */
export async function login(email: string, password: string): Promise<void> {
  const result = await betterAuthSignIn.email({
    email,
    password,
  });

  if (result.error) {
    throw new Error(result.error.message || 'Login failed');
  }
}

/**
 * Sign out current user
 */
export async function logout(): Promise<void> {
  await betterAuthSignOut();
}

/**
 * Sign in with Google
 */
export async function loginWithGoogle(): Promise<void> {
  await betterAuthSignIn.social({
    provider: 'google',
  });
}

/**
 * Reset password (if supported by Better Auth config)
 */
export async function resetPassword(email: string): Promise<void> {
  // This would require Better Auth to be configured with password reset
  // For now, just throw an error or implement when needed
  console.warn('Password reset not yet implemented with Better Auth');
  throw new Error('Password reset not yet implemented');
}

/**
 * Update user profile
 */
export async function updateUserProfile(displayName?: string, photoURL?: string): Promise<void> {
  // This would require Better Auth API call to update profile
  console.warn('Profile update not yet implemented');
  throw new Error('Profile update not yet implemented');
}

/**
 * Get current user from session
 */
export async function getCurrentUser(): Promise<User | null> {
  const { data: session } = await authClient.getSession();

  if (!session?.user) {
    return null;
  }

  return {
    id: session.user.id,
    email: session.user.email,
    name: session.user.name || session.user.email.split('@')[0],
    emailVerified: session.user.emailVerified || false,
    image: session.user.image,
    createdAt: session.user.createdAt || new Date(),
  };
}

/**
 * Get auth token for API requests
 */
export async function getAuthToken(): Promise<string | null> {
  const { data: session } = await authClient.getSession();
  return session?.session?.token || null;
}

/**
 * Check if user is authenticated
 */
export async function isAuthenticated(): Promise<boolean> {
  const { data: session } = await authClient.getSession();
  return !!session?.user;
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
