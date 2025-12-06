import React, { useState } from 'react';
import { useAuth } from '../../context/AuthContext';
import { useHistory } from '@docusaurus/router';
import styles from './Auth.module.css';

export default function UserProfile() {
  const { currentUser, logout } = useAuth();
  const [loading, setLoading] = useState(false);
  const history = useHistory();

  async function handleLogout() {
    setLoading(true);
    try {
      await logout();
      history.push('/');
    } catch (error) {
      console.error('Failed to log out', error);
    } finally {
      setLoading(false);
    }
  }

  if (!currentUser) {
    return null;
  }

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2 className={styles.authTitle}>Profile</h2>
        <p className={styles.authSubtitle}>Manage your account settings</p>

        <div style={{marginBottom: '2rem'}}>
          <div style={{
            display: 'flex',
            alignItems: 'center',
            gap: '1rem',
            marginBottom: '1.5rem',
            padding: '1.5rem',
            background: 'var(--ifm-color-emphasis-100)',
            borderRadius: '8px'
          }}>
            <div style={{
              width: '64px',
              height: '64px',
              borderRadius: '50%',
              background: 'var(--ifm-color-primary)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              color: 'white',
              fontSize: '1.5rem',
              fontWeight: 'bold'
            }}>
              {currentUser.displayName?.charAt(0).toUpperCase() || currentUser.email?.charAt(0).toUpperCase()}
            </div>
            <div>
              <div style={{fontWeight: 'bold', fontSize: '1.25rem', marginBottom: '0.25rem'}}>
                {currentUser.displayName || 'User'}
              </div>
              <div style={{color: 'var(--ifm-color-emphasis-600)', fontSize: '0.95rem'}}>
                {currentUser.email}
              </div>
            </div>
          </div>

          <div style={{marginBottom: '1rem'}}>
            <div style={{
              padding: '1rem',
              background: 'var(--ifm-background-color)',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '8px'
            }}>
              <div style={{fontSize: '0.9rem', color: 'var(--ifm-color-emphasis-600)', marginBottom: '0.25rem'}}>
                Account Status
              </div>
              <div style={{fontWeight: '600'}}>
                {currentUser.emailVerified ? (
                  <span style={{color: '#22c55e'}}>✓ Verified</span>
                ) : (
                  <span style={{color: '#f59e0b'}}>⚠ Not Verified</span>
                )}
              </div>
            </div>
          </div>

          <div style={{marginBottom: '1rem'}}>
            <div style={{
              padding: '1rem',
              background: 'var(--ifm-background-color)',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '8px'
            }}>
              <div style={{fontSize: '0.9rem', color: 'var(--ifm-color-emphasis-600)', marginBottom: '0.25rem'}}>
                Member Since
              </div>
              <div style={{fontWeight: '600'}}>
                {currentUser.createdAt ?
                  new Date(currentUser.createdAt).toLocaleDateString('en-US', {
                    year: 'numeric',
                    month: 'long',
                    day: 'numeric'
                  }) : 'N/A'}
              </div>
            </div>
          </div>

          <div style={{marginBottom: '1rem'}}>
            <div style={{
              padding: '1rem',
              background: 'var(--ifm-background-color)',
              border: '1px solid var(--ifm-color-emphasis-300)',
              borderRadius: '8px'
            }}>
              <div style={{fontSize: '0.9rem', color: 'var(--ifm-color-emphasis-600)', marginBottom: '0.25rem'}}>
                Authentication Method
              </div>
              <div style={{fontWeight: '600'}}>
                {currentUser.providerData[0]?.providerId === 'google.com' ? 'Google' : 'Email/Password'}
              </div>
            </div>
          </div>
        </div>

        <button
          onClick={handleLogout}
          disabled={loading}
          className={styles.submitButton}
          style={{background: '#dc2626'}}
        >
          {loading ? 'Signing out...' : 'Sign Out'}
        </button>

        <div className={styles.authFooter}>
          <a href="/" className={styles.authLink}>
            ← Back to Home
          </a>
        </div>
      </div>
    </div>
  );
}
