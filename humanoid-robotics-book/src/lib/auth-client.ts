/**
 * Better Auth Client Configuration
 * Uses better-auth.com for authentication
 */
import { createAuthClient } from "better-auth/react";

// Better Auth client pointing to backend
const getBackendUrl = () => {
  // Use environment variable if available, otherwise fallback to localhost
  if (typeof window !== 'undefined') {
    // Client side
    return window.location.origin.replace(window.location.port, '8000');
  } else {
    // Server side (SSR)
    return process.env.REACT_APP_API_URL || process.env.API_URL || 'http://localhost:8000';
  }
};

export const authClient = createAuthClient({
  baseURL: getBackendUrl(), // Backend API
});

// Export Better Auth methods
export const {
  signIn,
  signUp,
  signOut,
  useSession,
} = authClient;
