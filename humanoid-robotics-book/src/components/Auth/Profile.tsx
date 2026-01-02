import React, { useState, useEffect } from 'react';
import { useAuth } from '../../context/AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';
import styles from './Auth.module.css';

function ProfileContent() {
  const { currentUser, logout, updateUserProfile } = useAuth();
  const [displayName, setDisplayName] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [language, setLanguage] = useState('en');
  const history = useHistory();
  const location = useLocation();

  useEffect(() => {
    if (currentUser) {
      setDisplayName(currentUser.name || '');
    } else {
      // Redirect to login if not authenticated
      const currentPath = location.pathname;
      const isUrdu = currentPath.startsWith('/ur/');
      const loginPath = isUrdu ? '/ur/login' : '/login';
      const redirectParam = encodeURIComponent(currentPath);
      history.push(`${loginPath}?redirect=${redirectParam}`);
    }

    // Detect current language from URL
    if (location.pathname.startsWith('/ur/')) {
      setLanguage('ur');
    }
  }, [currentUser, history, location]);

  async function handleUpdateProfile(e: React.FormEvent) {
    e.preventDefault();

    if (!displayName.trim()) {
      setError('Display name cannot be empty');
      return;
    }

    try {
      setError('');
      setSuccess('');
      setLoading(true);
      await updateUserProfile(displayName);
      setSuccess('Profile updated successfully!');
    } catch (err: any) {
      setError(err?.message || 'Failed to update profile');
    } finally {
      setLoading(false);
    }
  }

  async function handleLogout() {
    try {
      await logout();
      const isUrdu = location.pathname.startsWith('/ur/');
      history.push(isUrdu ? '/ur/' : '/');
    } catch (err: any) {
      setError(err?.message || 'Failed to log out');
    }
  }

  function handleLanguageChange() {
    const currentPath = location.pathname;
    if (currentPath.startsWith('/ur/')) {
      // Switch to English
      const englishPath = currentPath.replace('/ur', '') || '/';
      history.push(englishPath);
    } else {
      // Switch to Urdu
      const urduPath = `/ur${currentPath}`;
      history.push(urduPath);
    }
  }

  if (!currentUser) {
    return (
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <p>Redirecting to login...</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2 className={styles.authTitle}>
          {language === 'ur' ? 'پروفائل' : 'Profile'}
        </h2>
        <p className={styles.authSubtitle}>
          {language === 'ur'
            ? 'اپنی معلومات اور ترجیحات کا نظم کریں'
            : 'Manage your account information and preferences'}
        </p>

        {error && (
          <div className={styles.errorAlert}>
            <span className={styles.errorIcon}>⚠️</span>
            {error}
          </div>
        )}

        {success && (
          <div className={styles.successAlert}>
            <span className={styles.successIcon}>✓</span>
            {success}
          </div>
        )}

        {/* Account Information */}
        <div className={styles.profileSection}>
          <h3 className={styles.sectionTitle}>
            {language === 'ur' ? 'اکاؤنٹ کی معلومات' : 'Account Information'}
          </h3>

          <div className={styles.infoGroup}>
            <label className={styles.infoLabel}>
              {language === 'ur' ? 'ای میل' : 'Email'}
            </label>
            <div className={styles.infoValue}>{currentUser.email}</div>
          </div>

          <div className={styles.infoGroup}>
            <label className={styles.infoLabel}>
              {language === 'ur' ? 'تصدیق شدہ' : 'Email Verified'}
            </label>
            <div className={styles.infoValue}>
              {currentUser.emailVerified ?
                (language === 'ur' ? '✓ ہاں' : '✓ Yes') :
                (language === 'ur' ? '✗ نہیں' : '✗ No')}
            </div>
          </div>

          <div className={styles.infoGroup}>
            <label className={styles.infoLabel}>
              {language === 'ur' ? 'شامل ہونے کی تاریخ' : 'Member Since'}
            </label>
            <div className={styles.infoValue}>
              {new Date(currentUser.createdAt).toLocaleDateString()}
            </div>
          </div>
        </div>

        {/* Update Display Name */}
        <form onSubmit={handleUpdateProfile} className={styles.authForm}>
          <div className={styles.formGroup}>
            <label className={styles.formLabel}>
              {language === 'ur' ? 'نام' : 'Display Name'}
            </label>
            <input
              type="text"
              className={styles.formInput}
              value={displayName}
              onChange={(e) => setDisplayName(e.target.value)}
              required
            />
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={loading}
          >
            {loading
              ? (language === 'ur' ? 'اپ ڈیٹ ہو رہا ہے...' : 'Updating...')
              : (language === 'ur' ? 'نام اپ ڈیٹ کریں' : 'Update Name')}
          </button>
        </form>

        {/* Preferences */}
        <div className={styles.profileSection}>
          <h3 className={styles.sectionTitle}>
            {language === 'ur' ? 'ترجیحات' : 'Preferences'}
          </h3>

          <div className={styles.preferenceGroup}>
            <label className={styles.preferenceLabel}>
              {language === 'ur' ? 'زبان' : 'Language'}
            </label>
            <button
              onClick={handleLanguageChange}
              className={styles.preferenceButton}
            >
              {language === 'ur' ? '🇬🇧 Switch to English' : '🇵🇰 اردو میں تبدیل کریں'}
            </button>
          </div>
        </div>

        {/* Logout */}
        <div className={styles.profileSection}>
          <button
            onClick={handleLogout}
            className={styles.logoutButton}
          >
            {language === 'ur' ? 'لاگ آؤٹ' : 'Log Out'}
          </button>
        </div>
      </div>
    </div>
  );
}

export default function Profile() {
  return (
    <BrowserOnly fallback={<div>Loading...</div>}>
      {() => <ProfileContent />}
    </BrowserOnly>
  );
}
