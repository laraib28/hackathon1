/**
 * Authentication Context - Better Auth Integration
 * Provides auth state and methods to all components
 */
import React, { createContext, useContext, ReactNode } from 'react';
import * as authService from '../services/authService';
import type { User } from '../services/authService';
import { useSession } from '../lib/auth-client';

interface AuthContextType {
  currentUser: User | null;
  loading: boolean;
  signup: (email: string, password: string, displayName?: string) => Promise<void>;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  resetPassword: (email: string) => Promise<void>;
  loginWithGoogle: () => Promise<void>;
  updateUserProfile: (displayName: string, photoURL?: string) => Promise<void>;
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
  const { data: session, isPending } = useSession();

  // Convert Better Auth session to our User format
  const currentUser: User | null = session?.user ? {
    id: session.user.id,
    email: session.user.email,
    name: session.user.name || session.user.email.split('@')[0],
    emailVerified: session.user.emailVerified || false,
    image: session.user.image,
    createdAt: session.user.createdAt || new Date(),
  } : null;

  async function signup(email: string, password: string, displayName?: string) {
    await authService.signup(email, password, displayName);
  }

  async function login(email: string, password: string) {
    await authService.login(email, password);
  }

  async function logout() {
    await authService.logout();
  }

  async function resetPassword(email: string) {
    await authService.resetPassword(email);
  }

  async function loginWithGoogle() {
    await authService.loginWithGoogle();
  }

  async function updateUserProfile(displayName: string, photoURL?: string) {
    await authService.updateUserProfile(displayName, photoURL);
  }

  const value: AuthContextType = {
    currentUser,
    loading: isPending,
    signup,
    login,
    logout,
    resetPassword,
    loginWithGoogle,
    updateUserProfile,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
}
