import React, { useState, useRef, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './ChatWidget.module.css';
import { useAuth } from '../../context/AuthContext';
import * as authService from '../../services/authService';

interface Message {
  id: string;
  text: string;
  isUser: boolean;
  language: 'en' | 'ur';
  timestamp: Date;
}

function detectLanguage(text: string): 'en' | 'ur' {
  const urduPattern = /[\u0600-\u06FF]/;
  return urduPattern.test(text) ? 'ur' : 'en';
}

export default function ChatWidget() {
  const { siteConfig } = useDocusaurusContext();
  const API_BASE_URL = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';
  const { currentUser, getUserPreferences, updateUserPreferences } = useAuth();
  const { pathname } = useLocation();
  // Detect site language from pathname
  const siteLanguage: 'en' | 'ur' = pathname.startsWith('/ur') ? 'ur' : 'en';
  const [isOpen, setIsOpen] = useState(false);

  // Get initial welcome message based on site language
  const getWelcomeMessage = () => {
    if (siteLanguage === 'ur') {
      return 'السلام علیکم! آج میں آپ کی کیسے مدد کر سکتا ہوں؟';
    }
    return 'Hello! How can I help you today?';
  };

  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: getWelcomeMessage(),
      isUser: false,
      language: siteLanguage,
      timestamp: new Date(),
    },
  ]);

  // Load chat history after component mounts (client-side only)
  // Reset messages when language changes
  useEffect(() => {
    const savedHistory = localStorage.getItem('chatHistory');
    const savedLanguage = localStorage.getItem('chatLanguage');

    // Only load history if it matches current language
    if (savedHistory && savedLanguage === siteLanguage) {
      try {
        const parsedHistory = JSON.parse(savedHistory, (key, value) => {
          if (key === 'timestamp') return new Date(value);
          return value;
        });
        setMessages(parsedHistory);
      } catch {
        // If parsing fails, reset to welcome message
        setMessages([
          {
            id: '1',
            text: getWelcomeMessage(),
            isUser: false,
            language: siteLanguage,
            timestamp: new Date(),
          },
        ]);
      }
    } else {
      // Language changed or no history - reset to welcome message
      setMessages([
        {
          id: '1',
          text: getWelcomeMessage(),
          isUser: false,
          language: siteLanguage,
          timestamp: new Date(),
        },
      ]);
      // Save current language
      if (typeof Storage !== 'undefined') {
        localStorage.setItem('chatLanguage', siteLanguage);
      }
    }
  }, [siteLanguage]);

  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const [chatHistoryEnabled, setChatHistoryEnabled] = useState(true);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Load user preferences on component mount
  useEffect(() => {
    const loadPreferences = async () => {
      if (currentUser) {
        const preferences = await getUserPreferences();
        if (preferences) {
          setChatHistoryEnabled(preferences.chatHistory !== false);
        }
      } else {
        // Use localStorage for non-authenticated users
        if (typeof Storage !== 'undefined') {
          const savedPref = localStorage.getItem('chatHistoryEnabled');
          if (savedPref !== null) {
            setChatHistoryEnabled(savedPref === 'true');
          }
        }
      }
    };

    loadPreferences();
  }, [currentUser]);

  // Listen for text selection event
  useEffect(() => {
    const handleOpenWithText = (event: any) => {
      const text = event.detail;
      if (text) {
        setSelectedText(text);
        setInputText(`Explain: "${text.substring(0, 100)}${text.length > 100 ? '...' : ''}"`);
        setIsOpen(true);
      }
    };

    window.addEventListener('openChatWithText', handleOpenWithText);
    return () => {
      window.removeEventListener('openChatWithText', handleOpenWithText);
    };
  }, []);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputText.trim()) return;

    const detectedLanguage = detectLanguage(inputText);

    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputText,
      isUser: true,
      language: detectedLanguage,
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Use FastAPI backend from Docusaurus config
      const API_ENDPOINT = `${API_BASE_URL}/api/chat`;

      // Use site language as target language if user didn't type in a specific language
const targetLanguage = siteLanguage;

      console.log('🔄 Sending request to:', API_ENDPOINT);
      console.log('📦 Request body:', {
        message: userMessage.text,
        target_language: targetLanguage,
        site_language: siteLanguage,
      });

      // Get auth token from localStorage (fallback to authService if needed)
      let authToken = localStorage.getItem('auth_token');
      if (!authToken && currentUser) {
        // Fallback to authService if token not in localStorage
        authToken = await authService.getAuthToken();
        if (authToken) {
          localStorage.setItem('auth_token', authToken);
        }
      }

      const headers: Record<string, string> = {
        'Content-Type': 'application/json',
      };

      // Add auth token to headers if available
      if (authToken) {
        headers['Authorization'] = `Bearer ${authToken}`;
      }

      const response = await fetch(API_ENDPOINT, {
        method: 'POST',
        headers,
        body: JSON.stringify({
          message: userMessage.text,
          target_language: targetLanguage,
          site_language: siteLanguage,
          selected_text: selectedText || null,
        }),
      });

      // Clear selected text after sending
      setSelectedText('');

      console.log('📡 Response status:', response.status);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('❌ Backend error:', response.status, errorText);
        throw new Error(`Backend returned ${response.status}: ${errorText}`);
      }

      const data = await response.json();
      console.log('✅ Response received:', data);

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: data.response || 'No response from server',
        isUser: false,
        language: targetLanguage,
        timestamp: new Date(),
      };

      const updatedMessages = [...messages, userMessage, botMessage];
      setMessages(updatedMessages);

      // Save chat history if enabled
      if (chatHistoryEnabled) {
        saveChatHistory(updatedMessages);
      }
    } catch (error) {
      console.error('❌ Fetch error:', error);

      // More specific error handling with better fallback responses
      // Use site language for error messages
      const errorLanguage = siteLanguage;
      let fallbackResponse = '';
      if (error instanceof TypeError && error.message.includes('fetch')) {
        // Network error
        fallbackResponse = errorLanguage === 'ur'
          ? 'نیٹ ورک کنکشن میں مسئلہ۔ کیا آپ کا سرور چل رہا ہے؟ http://localhost:8000 پر FastAPI سرور چلائیں۔'
          : 'Network connection issue. Is your server running? Start FastAPI server at http://localhost:8000.';
      } else if (error instanceof Error) {
        // General error
        fallbackResponse = errorLanguage === 'ur'
          ? 'معذرت، سرور اس وقت جواب نہیں دے رہا۔ براہ کرم بعد میں دوبارہ کوشش کریں۔'
          : 'Sorry, the server is not responding right now. Please try again later.';
      } else {
        // Unknown error
        fallbackResponse = errorLanguage === 'ur'
          ? 'کوئی خرابی ہو گئی۔ براہ کرم دوبارہ کوشش کریں۔'
          : 'An error occurred. Please try again.';
      }

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: fallbackResponse,
        isUser: false,
        language: errorLanguage,
        timestamp: new Date(),
      };

      const updatedMessages = [...messages, userMessage, botMessage];
      setMessages(updatedMessages);

      // Save chat history if enabled
      if (chatHistoryEnabled) {
        saveChatHistory(updatedMessages);
      }
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Function to save chat history to localStorage
  const saveChatHistory = (chatMessages: Message[]) => {
    try {
      // Check if localStorage is available (client-side only)
      if (typeof Storage !== 'undefined') {
        // Save only the last 50 messages to prevent localStorage from getting too large
        const messagesToSave = chatMessages.slice(-50);
        localStorage.setItem('chatHistory', JSON.stringify(messagesToSave));
        localStorage.setItem('chatLanguage', siteLanguage);
      }
    } catch (error) {
      console.error('Failed to save chat history:', error);
    }
  };

  // Function to clear chat history
  const clearChatHistory = () => {
    if (typeof Storage !== 'undefined') {
      localStorage.removeItem('chatHistory');
    }
    setMessages([
      {
        id: '1',
        text: getWelcomeMessage(),
        isUser: false,
        language: siteLanguage,
        timestamp: new Date(),
      },
    ]);
  };

  return (
    <div className={styles.chatWidget}>
      {!isOpen && (
        <button className={styles.chatButton} onClick={() => setIsOpen(true)}>
          💬
        </button>
      )}

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', width: '100%' }}>
              <h3>Robotics Assistant / روبوٹکس معاون</h3>
              <div style={{ display: 'flex', gap: '8px' }}>
                <button
                  className={styles.settingsButton}
                  onClick={() => {
                    // Toggle chat history setting
                    const newSetting = !chatHistoryEnabled;
                    setChatHistoryEnabled(newSetting);

                    // Save to localStorage for non-authenticated users
                    localStorage.setItem('chatHistoryEnabled', String(newSetting));

                    // Update user preferences if authenticated
                    if (currentUser) {
                      updateUserPreferences({ chatHistory: newSetting });
                    }
                  }}
                  title={chatHistoryEnabled ? 'Disable chat history' : 'Enable chat history'}
                  style={{
                    background: 'transparent',
                    border: 'none',
                    color: 'white',
                    cursor: 'pointer',
                    fontSize: '16px'
                  }}
                >
                  {chatHistoryEnabled ? '💾' : '📋'}
                </button>
                <button
                  className={styles.clearButton}
                  onClick={clearChatHistory}
                  title="Clear chat history"
                  style={{
                    background: 'transparent',
                    border: 'none',
                    color: 'white',
                    cursor: 'pointer',
                    fontSize: '16px'
                  }}
                >
                  🗑️
                </button>
                <button className={styles.closeButton} onClick={() => setIsOpen(false)}>
                  ✕
                </button>
              </div>
            </div>
          </div>
          {selectedText && (
            <div style={{
              padding: '8px 12px',
              backgroundColor: '#e3f2fd',
              borderBottom: '1px solid #ddd',
              fontSize: '12px',
              color: '#666'
            }}>
              📝 Selected text ({selectedText.length} chars)
            </div>
          )}

          <div className={styles.chatMessages}>
            {messages.map((msg) => (
              <div
                key={msg.id}
                className={`${styles.message} ${
                  msg.isUser ? styles.userMessage : styles.botMessage
                }`}
                dir={msg.language === 'ur' ? 'rtl' : 'ltr'}
              >
                <div className={styles.messageContent}>{msg.text}</div>
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span><span></span><span></span>
                  </div>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div className={styles.chatInput}>
            <textarea
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message... / اپنا پیغام ٹائپ کریں..."
              rows={1}
              className={styles.input}
            />

            <button
              className={styles.sendButton}
              onClick={handleSendMessage}
              disabled={!inputText.trim() || isLoading}
            >
              ➤
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
