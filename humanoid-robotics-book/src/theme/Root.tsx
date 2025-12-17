import React, { useState, useEffect } from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';

export default function Root({ children }: { children: React.ReactNode }) {
  const [selectedText, setSelectedText] = useState<string>('');
  const [showAskButton, setShowAskButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleSelectionChange = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim() || '';

      if (text.length > 0) {
        setSelectedText(text);

        // Get selection position
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setButtonPosition({
            x: rect.right + window.scrollX + 10,
            y: rect.bottom + window.scrollY + 5
          });
          setShowAskButton(true);
        }
      } else {
        setShowAskButton(false);
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);
    document.addEventListener('mouseup', handleSelectionChange);

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleSelectionChange);
    };
  }, []);

  const handleAskClick = () => {
    // Dispatch custom event to open chat with selected text
    const event = new CustomEvent('openChatWithText', { detail: selectedText });
    window.dispatchEvent(event);
    setShowAskButton(false);
    window.getSelection()?.removeAllRanges();
  };

  return (
    <>
      {children}

      {/* Ask button for text selection */}
      {showAskButton && (
        <button
          onClick={handleAskClick}
          style={{
            position: 'absolute',
            left: `${buttonPosition.x}px`,
            top: `${buttonPosition.y}px`,
            padding: '6px 12px',
            backgroundColor: '#0066cc',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            fontSize: '13px',
            fontWeight: '500',
            cursor: 'pointer',
            boxShadow: '0 2px 8px rgba(0,0,0,0.2)',
            zIndex: 9999,
            whiteSpace: 'nowrap'
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.backgroundColor = '#0052a3';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.backgroundColor = '#0066cc';
          }}
        >
          Ask
        </button>
      )}

      <ChatWidget />
    </>
  );
}
