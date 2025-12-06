import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import * as authService from '../services/authService';
import type { User } from '../services/authService';

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
  const [currentUser, setCurrentUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);

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

  useEffect(() => {
    // Set up auth state listener
    const unsubscribe = authService.onAuthStateChanged((user) => {
      setCurrentUser(user);
      setLoading(false);
    });

    return unsubscribe;
  }, []);

  const value: AuthContextType = {
    currentUser,
    loading,
    signup,
    login,
    logout,
    resetPassword,
    loginWithGoogle,
    updateUserProfile,
  };

  return (
    <AuthContext.Provider value={value}>
      {!loading && children}
    </AuthContext.Provider>
  );
}
