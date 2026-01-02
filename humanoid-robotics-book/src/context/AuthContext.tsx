/**
 * Authentication Context - Better Auth Integration
 * Provides auth state and methods to all components
 */
import React, { createContext, useContext, ReactNode, useState, useEffect } from 'react';
import * as authService from '../services/authService';
import type { User } from '../services/authService';
import { useSession } from '../lib/auth-client-safe';
import authLogger from '../services/auth-logger';

interface AuthContextType {
  currentUser: User | null;
  loading: boolean;
  signup: (email: string, password: string, displayName?: string) => Promise<void>;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  resetPassword: (email: string) => Promise<void>;
  loginWithGoogle: () => Promise<void>;
  updateUserProfile: (displayName: string, photoURL?: string) => Promise<void>;
  getUserPreferences: () => Promise<{ language?: string; theme?: string; chatHistory?: boolean } | null>;
  updateUserPreferences: (preferences: { language?: string; theme?: string; chatHistory?: boolean }) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

interface AuthProviderProps {
  children: ReactNode;
}

export function AuthProvider({ children }: AuthProviderProps) {
  // Use Better Auth's useSession hook
  const { data: session, isPending, error } = useSession();

  // Fallback state in case of auth service errors
  const [fallbackLoading, setFallbackLoading] = useState(true);
  const [fallbackUser, setFallbackUser] = useState<User | null>(null);

  // Convert Better Auth session to our User format
  const currentUser: User | null = session?.user ? {
    id: session.user.id,
    email: session.user.email,
    name: session.user.name || session.user.email.split('@')[0],
    emailVerified: session.user.emailVerified || false,
    image: session.user.image,
    createdAt: session.user.createdAt || new Date(),
  } : null;

  // Handle potential auth service errors gracefully
  useEffect(() => {
    if (error) {
      // If there's an auth error, we'll still allow the app to function
      // but with no authenticated user
      console.warn('Authentication service error:', error);
      authLogger.logSessionExpired();
      setFallbackLoading(false);
      setFallbackUser(null);
    } else {
      setFallbackLoading(false);
      // Convert session to user format only when session changes
      const user: User | null = session?.user ? {
        id: session.user.id,
        email: session.user.email,
        name: session.user.name || session.user.email.split('@')[0],
        emailVerified: session.user.emailVerified || false,
        image: session.user.image,
        createdAt: session.user.createdAt || new Date(),
      } : null;
      setFallbackUser(user);
    }
  }, [error, session]); // Use session instead of currentUser to avoid infinite loop

  async function signup(email: string, password: string, displayName?: string) {
    try {
      await authService.signup(email, password, displayName);
    } catch (error) {
      // If signup fails due to auth service being down, we'll throw the error
      // but the UI should handle it gracefully
      console.error('Signup failed:', error);
      throw error;
    }
  }

  async function login(email: string, password: string) {
    try {
      await authService.login(email, password);
    } catch (error) {
      // If login fails due to auth service being down, we'll throw the error
      // but the UI should handle it gracefully
      console.error('Login failed:', error);
      throw error;
    }
  }

  async function logout() {
    try {
      await authService.logout();
    } catch (error) {
      // If logout fails, we'll continue anyway to prevent blocking the user
      console.error('Logout failed:', error);
    }
  }

  async function resetPassword(email: string) {
    try {
      await authService.resetPassword(email);
    } catch (error) {
      console.error('Password reset failed:', error);
      throw error;
    }
  }

  async function loginWithGoogle() {
    try {
      await authService.loginWithGoogle();
    } catch (error) {
      console.error('Google login failed:', error);
      throw error;
    }
  }

  async function updateUserProfile(displayName: string, photoURL?: string) {
    try {
      await authService.updateUserProfile(displayName, photoURL);
    } catch (error) {
      console.error('Update profile failed:', error);
      throw error;
    }
  }

  async function getUserPreferences() {
    try {
      return await authService.getUserPreferences();
    } catch (error) {
      console.error('Get user preferences failed:', error);
      // Return default preferences in case of error
      return { language: 'en', theme: 'system' };
    }
  }

  async function updateUserPreferences(preferences: { language?: string; theme?: string; chatHistory?: boolean }) {
    try {
      await authService.updateUserPreferences(preferences);
    } catch (error) {
      console.error('Update user preferences failed:', error);
      throw error;
    }
  }

  // Use fallback values if there's an auth service error
  const finalUser = error ? fallbackUser : currentUser;
  const finalLoading = error ? fallbackLoading : isPending;

  const value: AuthContextType = {
    currentUser: finalUser,
    loading: finalLoading,
    signup,
    login,
    logout,
    resetPassword,
    loginWithGoogle,
    updateUserProfile,
    getUserPreferences,
    updateUserPreferences,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
}
