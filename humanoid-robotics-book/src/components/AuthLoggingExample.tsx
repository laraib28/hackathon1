/**
 * Example authentication component that demonstrates authentication logging
 */

import React, { useState } from 'react';
import { useAuth } from '../context/AuthContext';
import authLogger, { AuthEvent } from '../services/auth-logger';

const AuthLoggingExample: React.FC = () => {
  const { currentUser, login, signup, logout, loginWithGoogle } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    
    try {
      await login(email, password);
      authLogger.logAuthEvent(AuthEvent.LOGIN_SUCCESS, currentUser as any, {
        method: 'email_password',
        userAgent: navigator.userAgent,
        ip: 'CLIENT_IP_PLACEHOLDER' // In a real app, this would come from backend
      }, 'AuthLoggingExample');
    } catch (err) {
      setError('Login failed. Please check your credentials.');
      authLogger.logAuthEvent(AuthEvent.LOGIN_FAILURE, undefined, {
        method: 'email_password',
        error: err instanceof Error ? err.message : 'Unknown error',
        email
      }, 'AuthLoggingExample');
    } finally {
      setLoading(false);
    }
  };

  const handleSignup = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);
    
    try {
      await signup(email, password, name);
      authLogger.logAuthEvent(AuthEvent.SIGNUP_SUCCESS, currentUser as any, {
        method: 'email_password',
        userAgent: navigator.userAgent,
        displayName: name
      }, 'AuthLoggingExample');
    } catch (err) {
      setError('Signup failed. Please try again.');
      authLogger.logAuthEvent(AuthEvent.SIGNUP_FAILURE, undefined, {
        method: 'email_password',
        error: err instanceof Error ? err.message : 'Unknown error',
        email,
        displayName: name
      }, 'AuthLoggingExample');
    } finally {
      setLoading(false);
    }
  };

  const handleGoogleLogin = async () => {
    setLoading(true);
    setError(null);
    
    try {
      await loginWithGoogle();
      authLogger.logAuthEvent(AuthEvent.LOGIN_SUCCESS, currentUser as any, {
        method: 'google_oauth',
        userAgent: navigator.userAgent
      }, 'AuthLoggingExample');
    } catch (err) {
      setError('Google login failed. Please try again.');
      authLogger.logAuthEvent(AuthEvent.LOGIN_FAILURE, undefined, {
        method: 'google_oauth',
        error: err instanceof Error ? err.message : 'Unknown error'
      }, 'AuthLoggingExample');
    } finally {
      setLoading(false);
    }
  };

  const handleLogout = async () => {
    try {
      await logout();
      authLogger.logAuthEvent(AuthEvent.LOGOUT, currentUser as any, {
        userAgent: navigator.userAgent
      }, 'AuthLoggingExample');
    } catch (err) {
      setError('Logout failed. Please try again.');
    }
  };

  const viewAuthLogs = () => {
    // Since we can't access private properties, let's just demonstrate that logs exist
    authLogger.logAuthEvent(AuthEvent.SESSION_RENEWED, currentUser as any, {
      action: 'view_auth_logs',
      timestamp: new Date().toISOString()
    }, 'AuthLoggingExample');

    alert('Authentication logs are being captured. Check browser console for logs.');
  };

  return (
    <div className="auth-logging-example" style={{ padding: '20px', maxWidth: '500px', margin: '0 auto' }}>
      <h2>Authentication Logging Demo</h2>
      
      {error && (
        <div style={{ color: 'red', padding: '10px', backgroundColor: '#ffe6e6', borderRadius: '4px', marginBottom: '15px' }}>
          {error}
        </div>
      )}
      
      {currentUser ? (
        <div>
          <h3>Welcome, {currentUser.name}!</h3>
          <p>Logged in as: {currentUser.email}</p>
          <button 
            onClick={handleLogout} 
            style={{ 
              padding: '10px 15px', 
              backgroundColor: '#d9534f', 
              color: 'white', 
              border: 'none', 
              borderRadius: '4px',
              cursor: 'pointer',
              marginRight: '10px'
            }}
            disabled={loading}
          >
            {loading ? 'Logging out...' : 'Logout'}
          </button>
          <button 
            onClick={viewAuthLogs} 
            style={{ 
              padding: '10px 15px', 
              backgroundColor: '#5bc0de', 
              color: 'white', 
              border: 'none', 
              borderRadius: '4px',
              cursor: 'pointer'
            }}
            disabled={loading}
          >
            View Auth Logs
          </button>
        </div>
      ) : (
        <div>
          <h3>Login</h3>
          <form onSubmit={handleLogin} style={{ marginBottom: '20px' }}>
            <div style={{ marginBottom: '10px' }}>
              <input
                type="email"
                placeholder="Email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                style={{ width: '100%', padding: '8px', marginBottom: '5px' }}
                required
              />
            </div>
            <div style={{ marginBottom: '10px' }}>
              <input
                type="password"
                placeholder="Password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                style={{ width: '100%', padding: '8px', marginBottom: '5px' }}
                required
              />
            </div>
            <button 
              type="submit" 
              style={{ 
                padding: '10px 15px', 
                backgroundColor: '#5cb85c', 
                color: 'white', 
                border: 'none', 
                borderRadius: '4px',
                cursor: 'pointer'
              }}
              disabled={loading}
            >
              {loading ? 'Logging in...' : 'Login'}
            </button>
          </form>
          
          <h3>Sign Up</h3>
          <form onSubmit={handleSignup} style={{ marginBottom: '20px' }}>
            <div style={{ marginBottom: '10px' }}>
              <input
                type="text"
                placeholder="Full Name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                style={{ width: '100%', padding: '8px', marginBottom: '5px' }}
                required
              />
            </div>
            <div style={{ marginBottom: '10px' }}>
              <input
                type="email"
                placeholder="Email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                style={{ width: '100%', padding: '8px', marginBottom: '5px' }}
                required
              />
            </div>
            <div style={{ marginBottom: '10px' }}>
              <input
                type="password"
                placeholder="Password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                style={{ width: '100%', padding: '8px', marginBottom: '5px' }}
                required
              />
            </div>
            <button 
              type="submit" 
              style={{ 
                padding: '10px 15px', 
                backgroundColor: '#428bca', 
                color: 'white', 
                border: 'none', 
                borderRadius: '4px',
                cursor: 'pointer'
              }}
              disabled={loading}
            >
              {loading ? 'Signing up...' : 'Sign Up'}
            </button>
          </form>
          
          <button 
            onClick={handleGoogleLogin}
            style={{ 
              padding: '10px 15px', 
              backgroundColor: '#dd4b39', 
              color: 'white', 
              border: 'none', 
              borderRadius: '4px',
              cursor: 'pointer',
              width: '100%'
            }}
            disabled={loading}
          >
            {loading ? 'Logging in...' : 'Login with Google'}
          </button>
        </div>
      )}
      
      <div style={{ marginTop: '20px', padding: '15px', backgroundColor: '#f5f5f5', borderRadius: '5px' }}>
        <h3>How Authentication Logging Works:</h3>
        <p>Our authentication system automatically logs:</p>
        <ul>
          <li>Login attempts (success/failure)</li>
          <li>Sign up events (success/failure)</li>
          <li>Logout events</li>
          <li>Session expiration</li>
          <li>Password reset requests</li>
        </ul>
        <p>All logs include user context and additional metadata for security analysis.</p>
      </div>
    </div>
  );
};

export default AuthLoggingExample;