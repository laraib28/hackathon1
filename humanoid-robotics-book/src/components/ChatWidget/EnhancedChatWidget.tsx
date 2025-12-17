import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

const API_BASE = process.env.NODE_ENV === 'production'
  ? 'https://your-backend-url.com'
  : 'http://localhost:8000';

interface Message {
  id: string;
  text: string;
  isUser: boolean;
  language: 'en' | 'ur';
  timestamp: Date;
  sources?: any[];
}

function detectLanguage(text: string): 'en' | 'ur' {
  const urduPattern = /[\u0600-\u06FF]/;
  return urduPattern.test(text) ? 'ur' : 'en';
}

export default function EnhancedChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! I can help you understand the book content. Select any text and ask me questions about it! / السلام علیکم! میں کتاب کو سمجھنے میں آپ کی مدد کر سکتا ہوں۔ کسی بھی متن کو منتخب کریں اور مجھ سے سوال پوچھیں!',
      isUser: false,
      language: 'en',
      timestamp: new Date(),
    },
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [showSourcesFor, setShowSourcesFor] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Listen for text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 10 && text.length < 1000) {
        setSelectedText(text);
        if (!isOpen) {
          setIsOpen(true);
        }
        // Add a helper message
        const helperMessage: Message = {
          id: `helper-${Date.now()}`,
          text: `📝 Text selected: "${text.substring(0, 100)}${text.length > 100 ? '...' : ''}"\n\nAsk me anything about this text!`,
          isUser: false,
          language: 'en',
          timestamp: new Date(),
        };
        setMessages(prev => {
          const lastMsg = prev[prev.length - 1];
          if (lastMsg?.id.startsWith('helper-')) {
            return [...prev.slice(0, -1), helperMessage];
          }
          return [...prev, helperMessage];
        });
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, [isOpen]);

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
    const currentInput = inputText;
    const currentSelectedText = selectedText;
    setInputText('');
    setIsLoading(true);

    try {
      // Build conversation history
      const conversationHistory = messages
        .filter(m => m.isUser || !m.id.startsWith('helper-'))
        .slice(-6)
        .map(m => ({
          role: m.isUser ? 'user' : 'assistant',
          content: m.text
        }));

      const response = await fetch(`${API_BASE}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: currentInput,
          target_language: detectedLanguage,
          selected_text: currentSelectedText || null,
          conversation_history: conversationHistory
        }),
      });

      if (response.ok) {
        const data = await response.json();
        const botMessage: Message = {
          id: (Date.now() + 1).toString(),
          text: data.response,
          isUser: false,
          language: data.language,
          timestamp: new Date(),
          sources: data.sources
        };
        setMessages((prev) => [...prev, botMessage]);

        // Clear selected text after use
        if (currentSelectedText) {
          setSelectedText('');
        }
      } else {
        throw new Error('Failed to get response');
      }
    } catch (error) {
      console.error('Chat error:', error);
      const fallbackResponse = detectedLanguage === 'ur'
        ? 'معذرت، میں ابھی آپ کے سوال کا جواب نہیں دے سکتا۔ براہ کرم بعد میں دوبارہ کوشش کریں۔'
        : 'Sorry, I cannot answer your question right now. The backend API might not be running. Please try again later.';

      const botMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: fallbackResponse,
        isUser: false,
        language: detectedLanguage,
        timestamp: new Date(),
      };
      setMessages((prev) => [...prev, botMessage]);
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

  const toggleSources = (messageId: string) => {
    setShowSourcesFor(showSourcesFor === messageId ? null : messageId);
  };

  return (
    <div className={styles.chatWidget}>
      {!isOpen && (
        <button
          className={styles.chatButton}
          onClick={() => setIsOpen(true)}
          aria-label="Open chat"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            xmlns="http://www.w3.org/2000/svg"
          >
            <path
              d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2ZM20 16H6L4 18V4H20V16Z"
              fill="currentColor"
            />
            <path d="M7 9H17V11H7V9ZM7 12H14V14H7V12ZM7 6H17V8H7V6Z" fill="currentColor" />
          </svg>
        </button>
      )}

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div>
              <h3>RAG Chatbot / روبوٹکس معاون</h3>
              <small style={{ fontSize: '11px', opacity: 0.8 }}>💡 Select text to ask specific questions</small>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div key={message.id}>
                <div
                  className={`${styles.message} ${
                    message.isUser ? styles.userMessage : styles.botMessage
                  }`}
                  dir={message.language === 'ur' ? 'rtl' : 'ltr'}
                >
                  <div className={styles.messageContent}>{message.text}</div>
                  <div className={styles.messageTime}>
                    {message.timestamp.toLocaleTimeString([], {
                      hour: '2-digit',
                      minute: '2-digit',
                    })}
                  </div>
                </div>

                {/* Show sources if available */}
                {!message.isUser && message.sources && message.sources.length > 0 && (
                  <div className={styles.sourcesContainer}>
                    <button
                      className={styles.sourcesToggle}
                      onClick={() => toggleSources(message.id)}
                    >
                      📚 {showSourcesFor === message.id ? 'Hide' : 'Show'} Sources ({message.sources.length})
                    </button>

                    {showSourcesFor === message.id && (
                      <div className={styles.sourcesList}>
                        {message.sources.map((source, idx) => (
                          <div key={idx} className={styles.sourceItem}>
                            <a href={source.url} target="_blank" rel="noopener noreferrer">
                              📄 Source {idx + 1} (Score: {(source.score * 100).toFixed(0)}%)
                            </a>
                            <p>{source.content_preview}</p>
                          </div>
                        ))}
                      </div>
                    )}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.messageContent}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div className={styles.selectedTextBanner}>
              📝 Selected: {selectedText.substring(0, 60)}...
              <button onClick={() => setSelectedText('')} className={styles.clearSelection}>✕</button>
            </div>
          )}

          <div className={styles.chatInput}>
            <textarea
              value={inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={selectedText
                ? "Ask about the selected text..."
                : "Type your message... / اپنا پیغام ٹائپ کریں..."
              }
              className={styles.input}
              rows={1}
              disabled={isLoading}
            />
            <button
              onClick={handleSendMessage}
              className={styles.sendButton}
              disabled={!inputText.trim() || isLoading}
              aria-label="Send message"
            >
              <svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                xmlns="http://www.w3.org/2000/svg"
              >
                <path
                  d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z"
                  fill="currentColor"
                />
              </svg>
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
