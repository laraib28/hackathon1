// Simple Local Authentication Service
// Uses localStorage for demo/testing purposes
// No external dependencies required!

export interface User {
  uid: string;
  email: string;
  displayName: string;
  emailVerified: boolean;
  photoURL?: string;
  createdAt: string;
  providerData: Array<{
    providerId: string;
    uid: string;
    displayName: string;
    email: string;
  }>;
}

const USERS_KEY = 'auth_users';
const CURRENT_USER_KEY = 'auth_current_user';

// Get all users from localStorage
function getAllUsers(): Record<string, any> {
  if (typeof window === 'undefined') return {};
  const users = localStorage.getItem(USERS_KEY);
  return users ? JSON.parse(users) : {};
}

// Save users to localStorage
function saveUsers(users: Record<string, any>): void {
  if (typeof window === 'undefined') return;
  localStorage.setItem(USERS_KEY, JSON.stringify(users));
}

// Get current user
export function getCurrentUser(): User | null {
  if (typeof window === 'undefined') return null;
  const userJson = localStorage.getItem(CURRENT_USER_KEY);
  return userJson ? JSON.parse(userJson) : null;
}

// Set current user
function setCurrentUser(user: User | null): void {
  if (typeof window === 'undefined') return;
  if (user) {
    localStorage.setItem(CURRENT_USER_KEY, JSON.stringify(user));
  } else {
    localStorage.removeItem(CURRENT_USER_KEY);
  }
}

// Generate simple UID
function generateUID(): string {
  return Date.now().toString(36) + Math.random().toString(36).substring(2);
}

// Sign up with email and password
export async function signup(
  email: string,
  password: string,
  displayName?: string
): Promise<User> {
  if (typeof window === 'undefined') {
    throw new Error('Auth service only works in browser');
  }

  const users = getAllUsers();

  // Check if user already exists
  if (users[email]) {
    throw new Error('Email already in use');
  }

  // Create new user
  const user: User = {
    uid: generateUID(),
    email,
    displayName: displayName || email.split('@')[0],
    emailVerified: false,
    createdAt: new Date().toISOString(),
    providerData: [
      {
        providerId: 'password',
        uid: generateUID(),
        displayName: displayName || email.split('@')[0],
        email,
      },
    ],
  };

  // Save user with hashed password (simple demo hash)
  users[email] = {
    ...user,
    password: btoa(password), // Simple base64 encoding for demo
  };

  saveUsers(users);
  setCurrentUser(user);

  return user;
}

// Sign in with email and password
export async function login(email: string, password: string): Promise<User> {
  if (typeof window === 'undefined') {
    throw new Error('Auth service only works in browser');
  }

  const users = getAllUsers();
  const userData = users[email];

  if (!userData) {
    throw new Error('User not found');
  }

  // Verify password (simple demo verification)
  if (userData.password !== btoa(password)) {
    throw new Error('Invalid password');
  }

  // Create user object (without password)
  const { password: _, ...user } = userData;
  setCurrentUser(user as User);

  return user as User;
}

// Sign out
export async function logout(): Promise<void> {
  setCurrentUser(null);
}

// Simulate Google login (demo mode)
export async function loginWithGoogle(): Promise<User> {
  if (typeof window === 'undefined') {
    throw new Error('Auth service only works in browser');
  }

  // Create demo Google user
  const email = 'demo.google@example.com';
  const users = getAllUsers();

  let user = users[email];

  if (!user) {
    user = {
      uid: generateUID(),
      email,
      displayName: 'Google Demo User',
      emailVerified: true,
      photoURL: 'https://via.placeholder.com/150',
      createdAt: new Date().toISOString(),
      providerData: [
        {
          providerId: 'google.com',
          uid: 'google_' + generateUID(),
          displayName: 'Google Demo User',
          email,
        },
      ],
      password: btoa('demo123'), // Demo password
    };

    users[email] = user;
    saveUsers(users);
  }

  const { password: _, ...userWithoutPassword } = user;
  setCurrentUser(userWithoutPassword as User);

  return userWithoutPassword as User;
}

// Update user profile
export async function updateUserProfile(
  displayName?: string,
  photoURL?: string
): Promise<void> {
  const currentUser = getCurrentUser();
  if (!currentUser) {
    throw new Error('No user logged in');
  }

  const users = getAllUsers();
  const userData = users[currentUser.email];

  if (userData) {
    if (displayName) {
      userData.displayName = displayName;
      currentUser.displayName = displayName;
    }
    if (photoURL !== undefined) {
      userData.photoURL = photoURL;
      currentUser.photoURL = photoURL;
    }

    saveUsers(users);
    setCurrentUser(currentUser);
  }
}

// Password reset (demo - just logs message)
export async function resetPassword(email: string): Promise<void> {
  if (typeof window === 'undefined') {
    throw new Error('Auth service only works in browser');
  }

  const users = getAllUsers();
  if (!users[email]) {
    throw new Error('User not found');
  }

  // In a real app, this would send an email
  console.log(`Password reset link sent to ${email}`);
  alert(`Demo mode: Password reset link would be sent to ${email}\n\nFor demo purposes, you can create a new account or use:\nEmail: demo.google@example.com\nPassword: demo123`);
}

// Auth state change listener
export type AuthStateChangeCallback = (user: User | null) => void;

const listeners: Set<AuthStateChangeCallback> = new Set();

export function onAuthStateChanged(callback: AuthStateChangeCallback): () => void {
  // Immediately call with current user
  if (typeof window !== 'undefined') {
    setTimeout(() => callback(getCurrentUser()), 0);
  }

  // Add listener
  listeners.add(callback);

  // Return unsubscribe function
  return () => {
    listeners.delete(callback);
  };
}

// Notify all listeners
function notifyListeners(): void {
  const user = getCurrentUser();
  listeners.forEach((callback) => callback(user));
}

// Override setCurrentUser to notify listeners
const originalSetCurrentUser = setCurrentUser;
setCurrentUser = (user: User | null) => {
  originalSetCurrentUser(user);
  notifyListeners();
};
