import React, { useState } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import styles from './Auth.module.css';

const API_BASE = process.env.NODE_ENV === 'production'
  ? 'https://your-backend-url.com'
  : 'http://localhost:8001';

export default function EnhancedSignup() {
  const [step, setStep] = useState(1);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    name: '',
    software_experience: 'intermediate',
    hardware_experience: 'basic',
    programming_level: 'intermediate',
    programming_languages: [] as string[],
    learning_goals: '',
    industry_background: ''
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const history = useHistory();
  const location = useLocation();

  // Function to get locale-aware URL
  const getLocaleAwareUrl = (path: string): string => {
    const currentPath = location.pathname;
    // Check if we're currently on a Urdu page (starts with /ur/)
    if (currentPath.startsWith('/ur/')) {
      return `/ur${path}`;
    }
    // Check if we're on the root Urdu page
    if (currentPath === '/ur' || currentPath.startsWith('/ur?') || currentPath.startsWith('/ur#')) {
      // For signup/login from Urdu root, return to Urdu version
      if (path === '/signup') return '/ur/signup';
      if (path === '/login') return '/ur/login';
      return `/ur${path}`;
    }
    return path;
  };

  const programmingLanguages = [
    'Python', 'JavaScript', 'TypeScript', 'C++', 'Java',
    'C#', 'Go', 'Rust', 'MATLAB', 'R'
  ];

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    setFormData({
      ...formData,
      [e.target.name]: e.target.value
    });
  };

  const toggleLanguage = (lang: string) => {
    const languages = formData.programming_languages;
    if (languages.includes(lang)) {
      setFormData({
        ...formData,
        programming_languages: languages.filter(l => l !== lang)
      });
    } else {
      setFormData({
        ...formData,
        programming_languages: [...languages, lang]
      });
    }
  };

  const handleNext = () => {
    if (step === 1) {
      if (!formData.email || !formData.password || !formData.name) {
        setError('Please fill in all required fields');
        return;
      }
      if (formData.password !== formData.confirmPassword) {
        setError('Passwords do not match');
        return;
      }
      if (formData.password.length < 6) {
        setError('Password must be at least 6 characters');
        return;
      }
    }

    setError('');
    setStep(step + 1);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const response = await fetch(`${API_BASE}/api/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: formData.email,
          password: formData.password,
          name: formData.name,
          software_experience: formData.software_experience,
          hardware_experience: formData.hardware_experience,
          programming_level: formData.programming_level,
          programming_languages: formData.programming_languages,
          learning_goals: formData.learning_goals,
          industry_background: formData.industry_background
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        // Store token and user data
        localStorage.setItem('auth_token', data.token);
        localStorage.setItem('user_data', JSON.stringify(data.user));

        // Redirect to homepage
        history.push('/');
      } else {
        setError(data.detail || 'Failed to create account');
      }
    } catch (err: any) {
      setError(err.message || 'Failed to create account. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2 className={styles.authTitle}>Create Account</h2>
        <p className={styles.authSubtitle}>
          Step {step} of 2 - {step === 1 ? 'Basic Information' : 'Background Information'}
        </p>

        {error && (
          <div className={styles.errorAlert}>
            <span className={styles.errorIcon}>⚠️</span>
            {error}
          </div>
        )}

        <form onSubmit={handleSubmit} className={styles.authForm}>
          {step === 1 && (
            <>
              <div className={styles.formGroup}>
                <label htmlFor="name" className={styles.formLabel}>
                  Full Name *
                </label>
                <input
                  type="text"
                  id="name"
                  name="name"
                  className={styles.formInput}
                  value={formData.name}
                  onChange={handleChange}
                  required
                  placeholder="John Doe"
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="email" className={styles.formLabel}>
                  Email Address *
                </label>
                <input
                  type="email"
                  id="email"
                  name="email"
                  className={styles.formInput}
                  value={formData.email}
                  onChange={handleChange}
                  required
                  placeholder="you@example.com"
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="password" className={styles.formLabel}>
                  Password *
                </label>
                <input
                  type="password"
                  id="password"
                  name="password"
                  className={styles.formInput}
                  value={formData.password}
                  onChange={handleChange}
                  required
                  minLength={6}
                  placeholder="••••••••"
                />
                <small className={styles.formHelp}>At least 6 characters</small>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="confirmPassword" className={styles.formLabel}>
                  Confirm Password *
                </label>
                <input
                  type="password"
                  id="confirmPassword"
                  name="confirmPassword"
                  className={styles.formInput}
                  value={formData.confirmPassword}
                  onChange={handleChange}
                  required
                  placeholder="••••••••"
                />
              </div>

              <button
                type="button"
                onClick={handleNext}
                className={styles.submitButton}
              >
                Next →
              </button>
            </>
          )}

          {step === 2 && (
            <>
              <div className={styles.formGroup}>
                <label htmlFor="software_experience" className={styles.formLabel}>
                  Software Development Experience *
                </label>
                <select
                  id="software_experience"
                  name="software_experience"
                  className={styles.formInput}
                  value={formData.software_experience}
                  onChange={handleChange}
                  required
                >
                  <option value="beginner">Beginner (0-1 years)</option>
                  <option value="intermediate">Intermediate (1-3 years)</option>
                  <option value="advanced">Advanced (3-5 years)</option>
                  <option value="expert">Expert (5+ years)</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="hardware_experience" className={styles.formLabel}>
                  Hardware/Robotics Experience *
                </label>
                <select
                  id="hardware_experience"
                  name="hardware_experience"
                  className={styles.formInput}
                  value={formData.hardware_experience}
                  onChange={handleChange}
                  required
                >
                  <option value="none">No experience</option>
                  <option value="basic">Basic (hobby projects)</option>
                  <option value="intermediate">Intermediate (professional work)</option>
                  <option value="advanced">Advanced (specialized expertise)</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="programming_level" className={styles.formLabel}>
                  Overall Programming Level *
                </label>
                <select
                  id="programming_level"
                  name="programming_level"
                  className={styles.formInput}
                  value={formData.programming_level}
                  onChange={handleChange}
                  required
                >
                  <option value="beginner">Beginner</option>
                  <option value="intermediate">Intermediate</option>
                  <option value="advanced">Advanced</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label className={styles.formLabel}>
                  Programming Languages (select all that apply)
                </label>
                <div style={{ display: 'grid', gridTemplateColumns: 'repeat(2, 1fr)', gap: '8px', marginTop: '8px' }}>
                  {programmingLanguages.map(lang => (
                    <label key={lang} style={{ display: 'flex', alignItems: 'center', cursor: 'pointer' }}>
                      <input
                        type="checkbox"
                        checked={formData.programming_languages.includes(lang)}
                        onChange={() => toggleLanguage(lang)}
                        style={{ marginRight: '8px' }}
                      />
                      <span>{lang}</span>
                    </label>
                  ))}
                </div>
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="learning_goals" className={styles.formLabel}>
                  Learning Goals *
                </label>
                <textarea
                  id="learning_goals"
                  name="learning_goals"
                  className={styles.formInput}
                  value={formData.learning_goals}
                  onChange={handleChange}
                  required
                  placeholder="What do you want to learn from this book?"
                  rows={3}
                  style={{ resize: 'vertical' }}
                />
              </div>

              <div className={styles.formGroup}>
                <label htmlFor="industry_background" className={styles.formLabel}>
                  Industry Background (optional)
                </label>
                <input
                  type="text"
                  id="industry_background"
                  name="industry_background"
                  className={styles.formInput}
                  value={formData.industry_background}
                  onChange={handleChange}
                  placeholder="e.g., Automotive, Healthcare, Research"
                />
              </div>

              <div style={{ display: 'flex', gap: '12px' }}>
                <button
                  type="button"
                  onClick={() => setStep(1)}
                  className={styles.submitButton}
                  style={{ flex: 1, background: '#666' }}
                >
                  ← Back
                </button>
                <button
                  type="submit"
                  disabled={loading}
                  className={styles.submitButton}
                  style={{ flex: 2 }}
                >
                  {loading ? 'Creating account...' : 'Create Account'}
                </button>
              </div>
            </>
          )}
        </form>

        <div className={styles.authFooter}>
          Already have an account?{' '}
          <a href={getLocaleAwareUrl('/login')} className={styles.authLink}>
            Sign in
          </a>
        </div>
      </div>
    </div>
  );
}
