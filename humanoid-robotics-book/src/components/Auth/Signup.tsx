import React, { useState } from 'react';
import { useAuth } from '../../context/AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';
import styles from './Auth.module.css';

export default function Signup() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [displayName, setDisplayName] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { signup, loginWithGoogle } = useAuth();

  // ✅ Docusaurus supported hooks
  const history = useHistory();
  const location = useLocation();

  // ✅ Redirect handling
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

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (password.length < 6) {
      setError('Password must be at least 6 characters');
      return;
    }

    try {
      setError('');
      setLoading(true);
      await signup(email, password, displayName);

      // ✅ Docusaurus navigation
      history.push(redirectPath);

    } catch (err: any) {
      setError(err?.message || 'Failed to create account');
    } finally {
      setLoading(false);
    }
  }

  async function handleGoogleSignup() {
    try {
      setError('');
      setLoading(true);
      await loginWithGoogle();

      // ✅ Docusaurus navigation
      history.push(redirectPath);

    } catch (err: any) {
      setError(err?.message || 'Failed to sign up with Google');
    } finally {
      setLoading(false);
    }
  }

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2 className={styles.authTitle}>Create Account</h2>
        <p className={styles.authSubtitle}>
          Start your robotics learning journey
        </p>

        {error && (
          <div className={styles.errorAlert}>
            <span className={styles.errorIcon}>⚠️</span>
            {error}
          </div>
        )}

        <form onSubmit={handleSubmit} className={styles.authForm}>
          <div className={styles.formGroup}>
            <label className={styles.formLabel}>Full Name</label>
            <input
              type="text"
              className={styles.formInput}
              value={displayName}
              onChange={(e) => setDisplayName(e.target.value)}
              required
            />
          </div>

          <div className={styles.formGroup}>
            <label className={styles.formLabel}>Email</label>
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
              minLength={6}
            />
          </div>

          <div className={styles.formGroup}>
            <label className={styles.formLabel}>Confirm Password</label>
            <input
              type="password"
              className={styles.formInput}
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              required
            />
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={loading}
          >
            {loading ? 'Creating account…' : 'Create Account'}
          </button>
        </form>

        <div className={styles.divider}>
          <span>or</span>
        </div>

        <button
          onClick={handleGoogleSignup}
          className={styles.googleButton}
          disabled={loading}
        >
          Continue with Google
        </button>

        <div className={styles.authFooter}>
          Already have an account?{' '}
          <a
            href={location.pathname.startsWith('/ur/') ? '/ur/login' : '/login'}
            className={styles.authLink}
          >
            Sign in
          </a>
        </div>
      </div>
    </div>
  );
}
