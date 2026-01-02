import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced floating chatbot assistant with RAG integration
export default function FloatingAssistant({ className = '', ...props }) {
  const [isOpen, setIsOpen] = useState(false);
  const [isMinimized, setIsMinimized] = useState(true);
  const [messages, setMessages] = useState([
    { id: 1, type: 'ai', content: 'Hello! I\'m your AI assistant for this textbook. Ask me anything about the content. I can only answer questions based on the book material.' }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    if (isOpen && isMinimized) {
      setIsMinimized(false);
    } else {
      setIsOpen(!isOpen);
      if (!isOpen) {
        setIsMinimized(false);
      }
    }
  };

  const minimizeChat = () => {
    setIsMinimized(true);
  };

  const getCurrentLessonContext = () => {
    // Extract lesson context from URL
    const path = window.location.pathname;
    if (path.includes('/docs/chapters/')) {
      const parts = path.split('/');
      if (parts.length >= 5) {
        const chapter = parts[3];
        const lesson = parts[4];
        return `${chapter}/${lesson}`;
      }
    }
    return 'general textbook content';
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!input.trim() || isLoading) return;

    const userMessage = { id: Date.now(), type: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Get current context
      const lessonContext = getCurrentLessonContext();

      // Call the RAG backend API (will be proxied to /api/v1/chatbot/query)
      const response = await fetch('/api/chatbot/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: input,
          context: lessonContext
        }),
      });

      if (response.ok) {
        const data = await response.json();
        const aiResponse = {
          id: Date.now() + 1,
          type: 'ai',
          content: data.response,
          sources: data.sources || [],
          confidence: data.confidence || null
        };
        setMessages(prev => [...prev, aiResponse]);
      } else {
        const aiResponse = {
          id: Date.now() + 1,
          type: 'ai',
          content: 'Sorry, I encountered an error processing your question. Please try again.',
        };
        setMessages(prev => [...prev, aiResponse]);
      }
    } catch (error) {
      console.error('Error calling chatbot API:', error);
      const aiResponse = {
        id: Date.now() + 1,
        type: 'ai',
        content: 'Sorry, I encountered a network error. Please check your connection and try again.',
      };
      setMessages(prev => [...prev, aiResponse]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={clsx(styles.chatbotContainer, className)} {...props}>
      {isOpen && !isMinimized && (
        <div className={styles.chatbotWindow}>
          <div className={styles.chatbotHeader}>
            <div className={styles.chatbotTitle}>AI Assistant</div>
            <div className={styles.chatbotSubtitle}>This Book Only</div>
            <button
              className={styles.chatbotMinimize}
              onClick={minimizeChat}
              aria-label="Minimize chat"
            >
              −
            </button>
          </div>
          <div className={styles.chatbotBody}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`chatbot-message ${styles.chatbotMessage} ${message.type === 'user' ? 'user-message' : 'ai-message'}`}
              >
                {message.content}
                {message.sources && message.sources.length > 0 && (
                  <div className={styles.chatbotSources}>
                    <details>
                      <summary>Sources:</summary>
                      <ul>
                        {message.sources.map((source, index) => (
                          <li key={index}>{source}</li>
                        ))}
                      </ul>
                    </details>
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.chatbotMessage} ai-message`}>
                <div className={styles.chatbotLoading}>
                  <div className="loading-dots">
                    <div></div>
                    <div></div>
                    <div></div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSendMessage} className={styles.chatbotInputContainer}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              className={styles.chatbotInput}
              placeholder="Ask a question about this lesson..."
              disabled={isLoading}
            />
            <button
              type="submit"
              className={styles.chatbotSend}
              disabled={!input.trim() || isLoading}
            >
              Send
            </button>
          </form>
        </div>
      )}
      <button
        className={clsx(styles.chatbotToggle, {
          [styles.chatbotToggleOpen]: isOpen,
          [styles.chatbotToggleMinimized]: isMinimized && isOpen,
        })}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isMinimized && isOpen ? (
          <span className={styles.chatbotToggleText}>AI</span>
        ) : (
          <span className={styles.chatbotToggleIcon}>💬</span>
        )}
        {isOpen && isMinimized && (
          <span className={styles.chatbotNotification}>{messages.length - 1}</span>
        )}
      </button>
    </div>
  );
}