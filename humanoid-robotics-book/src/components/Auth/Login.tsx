import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useAuth } from '../../context/AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';
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
  const history = useHistory();
  const location = useLocation();

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
      await login(email, password);
      history.push(redirectPath);
    } catch (err: any) {
      setError(err.message || 'Failed to log in');
    } finally {
      setLoading(false);
    }
  }

  async function handleGoogleLogin() {
    try {
      setError('');
      setLoading(true);
      await loginWithGoogle();
      history.push(redirectPath);
    } catch (err: any) {
      setError(err.message || 'Failed to log in with Google');
    } finally {
      setLoading(false);
    }
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
