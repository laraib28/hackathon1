import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import styles from './ChapterControls.module.css';

const API_BASE = process.env.NODE_ENV === 'production'
  ? 'https://your-backend-url.com'
  : 'http://localhost:8000';

interface ChapterControlsProps {
  chapterId: string;
  content: string;
  onContentUpdate?: (newContent: string) => void;
}

export default function ChapterControls({
  chapterId,
  content,
  onContentUpdate
}: ChapterControlsProps) {
  const [user, setUser] = useState<any>(null);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [loading, setLoading] = useState(false);
  const [originalContent, setOriginalContent] = useState(content);
  const location = useLocation();

  useEffect(() => {
    // Load user data from localStorage
    const userData = localStorage.getItem('user_data');
    if (userData) {
      setUser(JSON.parse(userData));
    }

    // Load translation state from localStorage
    const savedTranslationState = localStorage.getItem(`translation_state_${chapterId}`);
    if (savedTranslationState) {
      const { isTranslated: savedIsTranslated, translatedContent } = JSON.parse(savedTranslationState);
      if (savedIsTranslated && translatedContent) {
        setIsTranslated(savedIsTranslated);
        if (onContentUpdate) {
          onContentUpdate(translatedContent);
        }
      }
    }

    // Load personalization state from localStorage
    const savedPersonalizationState = localStorage.getItem(`personalization_state_${chapterId}`);
    if (savedPersonalizationState) {
      const { isPersonalized: savedIsPersonalized, personalizedContent } = JSON.parse(savedPersonalizationState);
      if (savedIsPersonalized && personalizedContent) {
        setIsPersonalized(savedIsPersonalized);
        if (onContentUpdate) {
          onContentUpdate(personalizedContent);
        }
      }
    }
  }, [chapterId, onContentUpdate]);

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

  const handlePersonalize = async () => {
    if (!user) {
      alert('Please log in to personalize content');
      return;
    }

    setLoading(true);

    try {
      const response = await fetch(`${API_BASE}/api/chat/personalize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: originalContent,
          user_background: {
            software_experience: user.software_experience,
            hardware_experience: user.hardware_experience,
            programming_level: user.programming_level,
            learning_goals: user.learning_goals
          }
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        setIsPersonalized(true);
        if (onContentUpdate) {
          onContentUpdate(data.personalized_content);
        }
        // Save personalization state to localStorage
        const personalizationState = {
          isPersonalized: true,
          personalizedContent: data.personalized_content
        };
        localStorage.setItem(`personalization_state_${chapterId}`, JSON.stringify(personalizationState));

        // Optionally, re-render the page with new content
        alert('✅ Content personalized based on your background!');
      } else {
        throw new Error('Personalization failed');
      }
    } catch (error) {
      console.error('Personalization error:', error);
      alert('Failed to personalize content. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleTranslate = async () => {
    setLoading(true);

    try {
      const targetLang = isTranslated ? 'en' : 'ur';

      const response = await fetch(`${API_BASE}/api/chat/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: originalContent,
          target_language: targetLang
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        const newIsTranslated = !isTranslated;
        setIsTranslated(newIsTranslated);
        if (onContentUpdate) {
          onContentUpdate(data.translated_text);
        }

        // Save translation state to localStorage
        const translationState = {
          isTranslated: newIsTranslated,
          translatedContent: newIsTranslated ? data.translated_text : null
        };
        localStorage.setItem(`translation_state_${chapterId}`, JSON.stringify(translationState));

        alert(`✅ Content ${isTranslated ? 'restored to English' : 'translated to Urdu'}!`);
      } else {
        throw new Error('Translation failed');
      }
    } catch (error) {
      console.error('Translation error:', error);
      alert('Failed to translate content. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleReset = () => {
    setIsPersonalized(false);
    setIsTranslated(false);
    if (onContentUpdate) {
      onContentUpdate(originalContent);
    }
    // Clear translation and personalization states from localStorage
    localStorage.removeItem(`translation_state_${chapterId}`);
    localStorage.removeItem(`personalization_state_${chapterId}`);
    alert('✅ Content reset to original!');
  };

  return (
    <div className={styles.controlsContainer}>
      <div className={styles.controlsCard}>
        <div className={styles.controlsHeader}>
          <h4>📖 Customize This Chapter</h4>
          {user && (
            <span className={styles.userBadge}>
              👤 {user.name}
            </span>
          )}
        </div>

        <div className={styles.controlsButtons}>
          <button
            className={`${styles.controlButton} ${isPersonalized ? styles.active : ''}`}
            onClick={handlePersonalize}
            disabled={loading || !user}
            title={!user ? 'Please log in to personalize' : 'Personalize based on your background'}
          >
            <span className={styles.buttonIcon}>🎯</span>
            <span className={styles.buttonText}>
              {isPersonalized ? 'Personalized' : 'Personalize for Me'}
            </span>
          </button>

          <button
            className={`${styles.controlButton} ${isTranslated ? styles.active : ''}`}
            onClick={handleTranslate}
            disabled={loading}
            title={isTranslated ? 'Restore to English' : 'Translate to Urdu'}
          >
            <span className={styles.buttonIcon}>🌐</span>
            <span className={styles.buttonText}>
              {isTranslated ? 'اردو (Urdu)' : 'Translate to Urdu'}
            </span>
          </button>

          {(isPersonalized || isTranslated) && (
            <button
              className={styles.controlButton}
              onClick={handleReset}
              disabled={loading}
              title="Reset to original content"
            >
              <span className={styles.buttonIcon}>↺</span>
              <span className={styles.buttonText}>Reset</span>
            </button>
          )}
        </div>

        {loading && (
          <div className={styles.loadingBar}>
            <div className={styles.loadingProgress}></div>
          </div>
        )}

        {!user && (
          <div className={styles.loginPrompt}>
            <p>
              <a href={getLocaleAwareUrl('/signup')} className={styles.loginLink}>Sign up</a> or{' '}
              <a href={getLocaleAwareUrl('/login')} className={styles.loginLink}>log in</a> to personalize content based on your background!
            </p>
          </div>
        )}

        {user && (
          <div className={styles.userInfo}>
            <small>
              Content adapted for: {user.programming_level} programmer with {user.software_experience} software experience
            </small>
          </div>
        )}
      </div>
    </div>
  );
}
