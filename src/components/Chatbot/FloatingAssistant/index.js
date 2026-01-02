import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

// Enhanced floating chatbot assistant with new visual design
export default function FloatingAssistant({ className = '', ...props }) {
  const [isOpen, setIsOpen] = useState(false);
  const [isMinimized, setIsMinimized] = useState(true);

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
            <div className={styles.chatbotMessage}>
              Hello! I'm your AI assistant for this textbook. Ask me anything about the content.
            </div>
          </div>
          <div className={styles.chatbotInputContainer}>
            <input
              type="text"
              className={styles.chatbotInput}
              placeholder="Ask a question about this lesson..."
            />
            <button className={styles.chatbotSend}>Send</button>
          </div>
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
          <span className={styles.chatbotNotification}>1</span>
        )}
      </button>
    </div>
  );
}