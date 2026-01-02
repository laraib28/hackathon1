import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useAuth } from '../../context/AuthContext';
import { useLocation, useHistory } from '@docusaurus/router';
import styles from './Auth.module.css';

export default function Login() {
  return (
    <BrowserOnly>
      {() => <LoginInner />}
    </BrowserOnly>
  );
}

function LoginInner() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { login, loginWithGoogle } = useAuth();
  const location = useLocation();
  const history = useHistory();

  // redirect handling
  const searchParams = new URLSearchParams(location.search);
  const redirectParam = searchParams.get('redirect');
  let redirectPath = '/';

  if (redirectParam) {
    const decodedRedirect = decodeURIComponent(redirectParam);
    if (location.pathname.startsWith('/ur/')) {
      redirectPath = decodedRedirect.startsWith('/ur')
        ? decodedRedirect
        : `/ur${decodedRedirect}`;
    } else {
      redirectPath = decodedRedirect;
    }
  }

  async function handleSubmit(e: React.FormEvent) {
    e.preventDefault();
    try {
      setError('');
      setLoading(true);

      console.log('📝 Submitting login form...');
      await login(email, password);
      console.log('✅ Login successful, redirecting...');

      // Redirect immediately after successful login
      // The session will be available on the next page
      window.location.href = redirectPath;
    } catch (err: any) {
      // Handle different error types gracefully
      if (err.message?.includes('fetch') || err.message?.includes('Failed to fetch')) {
        setError('Unable to connect to authentication server. Please check your connection and try again.');
      } else if (err.message?.includes('credentials') || err.message?.includes('INVALID_EMAIL_OR_PASSWORD')) {
        setError('Invalid email or password. Please try again.');
      } else {
        setError(err.message || 'Failed to log in. Please try again.');
      }
      console.error('Login error:', err);
      setLoading(false);
    }
  }

  async function handleGoogleLogin() {
    try {
      setError('');
      setLoading(true);

      console.log('📝 Submitting Google login...');
      await loginWithGoogle();
      console.log('✅ Google login successful, redirecting...');

      // Redirect immediately after successful login
      window.location.href = redirectPath;
    } catch (err: any) {
      // Handle different error types gracefully
      if (err.message?.includes('fetch') || err.message?.includes('Failed to fetch')) {
        setError('Unable to connect to authentication server. Please check your connection and try again.');
      } else {
        setError(err.message || 'Failed to log in with Google. Please try again.');
      }
      console.error('Google login error:', err);
      setLoading(false);
    }
  }

  // Show an error message if there's an auth service issue
  if (error && error.includes('fetch') && error.includes('failed')) {
    return (
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h2 className={styles.authTitle}>Authentication Service Unavailable</h2>
          <p className={styles.authSubtitle}>
            We're having trouble connecting to our authentication service.
          </p>

          <div className={styles.errorAlert}>
            <span className={styles.errorIcon}>⚠️</span>
            Authentication service is temporarily unavailable. Please try again later.
          </div>

          <div className={styles.authFooter}>
            <a href="/">Return to Home</a>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2 className={styles.authTitle}>Sign In</h2>
        <p className={styles.authSubtitle}>
          Access your robotics learning dashboard
        </p>

        {error && (
          <div className={styles.errorAlert}>
            <span className={styles.errorIcon}>⚠️</span>
            {error}
          </div>
        )}

        <form onSubmit={handleSubmit} className={styles.authForm}>
          <div className={styles.formGroup}>
            <label className={styles.formLabel}>Email Address</label>
            <input
              type="email"
              className={styles.formInput}
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />
          </div>

          <div className={styles.formGroup}>
            <label className={styles.formLabel}>Password</label>
            <input
              type="password"
              className={styles.formInput}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
          </div>

          <button
            type="submit"
            disabled={loading}
            className={styles.submitButton}
          >
            {loading ? 'Signing in...' : 'Sign In'}
          </button>
        </form>

        <div className={styles.divider}>or</div>

        <button
          onClick={handleGoogleLogin}
          disabled={loading}
          className={styles.googleButton}
        >
          Continue with Google
        </button>

        <div className={styles.authFooter}>
          Don’t have an account?{' '}
          <a href={location.pathname.startsWith('/ur/') ? '/ur/signup' : '/signup'}>
            Sign up
          </a>
        </div>
      </div>
    </div>
  );
}
