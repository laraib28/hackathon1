import React, { useState, useRef, useEffect } from 'react';
import styles from './ChatWidget.module.css';

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
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! How can I help you today? / السلام علیکم! آج میں آپ کی کیسے مدد کر سکتا ہوں؟',
      isUser: false,
      language: 'en',
      timestamp: new Date(),
    },
  ]);

  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState<string>('');
  const messagesEndRef = useRef<HTMLDivElement>(null);

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
      // ✅ FIXED: Always use http://localhost:8000/api/chat
      const API_URL = 'http://localhost:8000/api/chat';

      console.log('🔄 Sending POST request to:', API_URL);
      console.log('📦 Request body:', {
        message: userMessage.text,
        target_language: detectedLanguage,
      });

      const response = await fetch(API_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage.text,
          target_language: detectedLanguage,
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
        language: detectedLanguage,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      console.error('❌ Fetch error:', error);

      const fallbackResponse =
        detectedLanguage === 'ur'
          ? 'معذرت، سرور اس وقت جواب نہیں دے رہا۔ براہ کرم بعد میں دوبارہ کوشش کریں۔'
          : 'Sorry, the server is not responding right now. Please try again later.';

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
            <h3>Robotics Assistant / روبوٹکس معاون</h3>
            <button className={styles.closeButton} onClick={() => setIsOpen(false)}>
              ✕
            </button>
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
