/**
 * Better Auth Client Configuration
 * Uses better-auth.com for authentication
 */
import { createAuthClient } from "better-auth/react";

// Better Auth client pointing to backend
export const authClient = createAuthClient({
  baseURL: "http://localhost:8000", // Backend API
});

// Export Better Auth methods
export const {
  signIn,
  signUp,
  signOut,
  useSession,
} = authClient;
