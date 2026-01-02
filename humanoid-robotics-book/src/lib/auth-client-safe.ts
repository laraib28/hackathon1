/**
 * Better Auth Client Configuration - Safe Version
 * Uses better-auth.com for authentication with error handling
 */
import { createAuthClient } from "better-auth/react";
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Backend URL with fallback - using a simple approach that works in browser
// Since process.env is not available in browser, we use a default URL
// In Docusaurus, environment variables can be injected via the config
const BACKEND_URL = (() => {
  // Check if we're in browser and if there's a custom config
  if (typeof window !== 'undefined') {
    // Allow configuration via window.config or similar mechanism
    // This can be set in the Docusaurus config or HTML template
    return (window as any).AUTH_CONFIG?.BACKEND_URL || 'http://localhost:3001';
  }
  return 'http://localhost:3001';
})();

// Debug logging to verify URL
if (typeof window !== 'undefined') {
  console.log('🔧 Better Auth Client Configuration:');
  console.log('  Base URL:', BACKEND_URL);
  console.log('  Window location:', window.location.href);
}

// Create auth client with error handling and additional features
let authClientInstance: any = null;

try {
  authClientInstance = createAuthClient({
    baseURL: BACKEND_URL, // Backend API URL
    fetchOptions: {
      cache: "no-store", // Prevent caching issues
      credentials: "include", // Include cookies for CORS requests
    }
  });
} catch (error) {
  console.error('❌ Failed to initialize auth client:', error);
  console.warn('⚠️  Auth client disabled - using fallback methods');
  
  // Create a mock client that returns safe defaults
  authClientInstance = {
    getSession: async () => ({ data: null, error: null }),
    signIn: {
      email: async () => ({ error: { message: 'Auth service unavailable' } }),
      social: async () => ({ error: { message: 'Auth service unavailable' } })
    },
    signUp: {
      email: async () => ({ error: { message: 'Auth service unavailable' } })
    },
    signOut: async () => {},
  };
}

// Export Better Auth methods
export const authClient = authClientInstance;

export const {
  signIn,
  signUp,
  signOut,
  useSession,
} = authClient;

// Enhanced auth functions with personalization
// Note: Profile updates are handled by Better Auth's built-in methods
// For now, we use Better Auth's default user object which includes name and image

export const updateProfile = async (displayName?: string, photoURL?: string) => {
  try {
    const session = await authClient.getSession();
    if (!session.data?.user) {
      throw new Error('User not authenticated');
    }

    // Better Auth doesn't expose a direct profile update endpoint in the client
    // For now, we'll return the current user data
    // In production, implement a custom endpoint on the auth server
    console.log('Profile update requested:', { displayName, photoURL });

    return {
      success: true,
      message: 'Profile update functionality will be implemented with backend support',
      user: session.data.user
    };
  } catch (error) {
    console.error('Profile update error:', error);
    throw new Error('Failed to update profile');
  }
};

// User preferences stored in localStorage for now
// In production, implement backend storage
export const getUserPreferences = async () => {
  try {
    if (typeof window === 'undefined') {
      return { language: 'en', theme: 'system' };
    }

    const stored = localStorage.getItem('user_preferences');
    if (stored) {
      return JSON.parse(stored);
    }

    return { language: 'en', theme: 'system' };
  } catch (error) {
    console.error('Failed to get user preferences:', error);
    return { language: 'en', theme: 'system' };
  }
};

export const updateUserPreferences = async (preferences: { language?: string; theme?: string; chatHistory?: boolean }) => {
  try {
    if (typeof window === 'undefined') {
      return { success: false };
    }

    // Get existing preferences
    const existing = await getUserPreferences();
    const updated = { ...existing, ...preferences };

    // Store in localStorage
    localStorage.setItem('user_preferences', JSON.stringify(updated));

    return { success: true, preferences: updated };
  } catch (error) {
    console.error('Failed to update preferences:', error);
    throw new Error('Failed to update preferences');
  }
};