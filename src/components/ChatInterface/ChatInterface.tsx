/**
 * ChatInterface - Floating RAG chatbot widget
 *
 * Features:
 * - Floating trigger button (bottom-right)
 * - Expandable chat overlay
 * - Message history with citations
 * - Loading states
 * - Error handling
 * - Responsive design
 */

import React, { useState, useRef, useEffect } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import type { Message } from '@site/src/types/api';
import { chatWithRAG, APIClientError } from '@site/src/services/api';
import ChatMessage from './ChatMessage';
import styles from './ChatInterface.module.css';

export default function ChatInterface(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [conversationId, setConversationId] = useState<string | undefined>();
  const [error, setError] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  /**
   * Scroll to bottom of messages
   */
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  /**
   * Auto-scroll when messages change
   */
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  /**
   * Focus input when chat opens
   */
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  /**
   * Send message to RAG backend
   */
  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      id: `user-${Date.now()}`,
      role: 'user',
      content: input.trim(),
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await chatWithRAG(userMessage.content, conversationId);

      // Update conversation ID for context continuity
      if (!conversationId) {
        setConversationId(response.conversation_id);
      }

      const assistantMessage: Message = {
        id: `assistant-${Date.now()}`,
        role: 'assistant',
        content: response.response,
        sources: response.sources,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Error sending message:', err);

      let errorMessage = 'Sorry, I encountered an error. Please try again.';

      if (err instanceof APIClientError) {
        if (err.statusCode === 503) {
          errorMessage =
            'The AI service is currently unavailable. Please try again in a few moments.';
        } else if (err.statusCode === 400) {
          errorMessage = 'Invalid request. Please rephrase your question.';
        } else if (err.statusCode === 408) {
          errorMessage = 'Request timed out. Please try again with a shorter question.';
        } else {
          errorMessage = err.message;
        }
      }

      setError(errorMessage);

      const errorMessageObj: Message = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: errorMessage,
        timestamp: new Date(),
      };

      setMessages((prev) => [...prev, errorMessageObj]);
    } finally {
      setIsLoading(false);
    }
  };

  /**
   * Handle Enter key press (Shift+Enter for new line)
   */
  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  /**
   * Toggle chat open/closed
   */
  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  /**
   * Clear conversation
   */
  const clearConversation = () => {
    setMessages([]);
    setConversationId(undefined);
    setError(null);
  };

  return (
    <>
      {/* Floating trigger button */}
      <button
        className={`${styles.floatingButton} ${isOpen ? styles.hidden : ''}`}
        onClick={toggleChat}
        aria-label="Open chat"
        title="Ask the Physical AI Assistant"
      >
        <svg
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M20 2H4C2.9 2 2 2.9 2 4V22L6 18H20C21.1 18 22 17.1 22 16V4C22 2.9 21.1 2 20 2Z"
            fill="currentColor"
          />
        </svg>
      </button>

      {/* Chat overlay */}
      {isOpen && (
        <div className={styles.chatOverlay}>
          <div className={styles.chatContainer}>
            {/* Header */}
            <div className={styles.chatHeader}>
              <div className={styles.headerContent}>
                <h3 className={styles.headerTitle}>Physical AI Assistant</h3>
                <p className={styles.headerSubtitle}>
                  Ask about ROS2, Gazebo, Unity, NVIDIA Isaac, and more
                </p>
              </div>
              <div className={styles.headerActions}>
                {messages.length > 0 && (
                  <button
                    className={styles.clearButton}
                    onClick={clearConversation}
                    title="Clear conversation"
                    aria-label="Clear conversation"
                  >
                    <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
                      <path d="M2 3h12v1H2V3zm1 2h10v9a1 1 0 01-1 1H4a1 1 0 01-1-1V5zm3-3V1h4v1H6z" />
                    </svg>
                  </button>
                )}
                <button
                  className={styles.closeButton}
                  onClick={toggleChat}
                  aria-label="Close chat"
                >
                  ×
                </button>
              </div>
            </div>

            {/* Messages */}
            <div className={styles.messagesContainer}>
              {messages.length === 0 && (
                <div className={styles.emptyState}>
                  <div className={styles.emptyStateIcon}>💬</div>
                  <h4>Welcome to Physical AI Assistant!</h4>
                  <p>
                    I can help you understand concepts from the Physical AI book.
                    Ask me about:
                  </p>
                  <ul className={styles.exampleList}>
                    <li>ROS2 fundamentals and architecture</li>
                    <li>Gazebo and Unity simulation</li>
                    <li>NVIDIA Isaac platform</li>
                    <li>Vision-Language-Action models</li>
                  </ul>
                </div>
              )}

              {messages.map((message) => (
                <ChatMessage key={message.id} message={message} />
              ))}

              {/* Loading indicator */}
              {isLoading && (
                <div className={`${styles.message} ${styles.assistantMessage}`}>
                  <div className={styles.messageRole}>Physical AI Assistant</div>
                  <div className={styles.loadingDots}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              )}

              <div ref={messagesEndRef} />
            </div>

            {/* Input */}
            <div className={styles.inputContainer}>
              <textarea
                ref={inputRef}
                className={styles.input}
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask a question about Physical AI..."
                rows={2}
                disabled={isLoading}
                maxLength={500}
              />
              <button
                className={styles.sendButton}
                onClick={sendMessage}
                disabled={isLoading || !input.trim()}
                aria-label="Send message"
              >
                {isLoading ? (
                  'Sending...'
                ) : (
                  <svg
                    width="20"
                    height="20"
                    viewBox="0 0 20 20"
                    fill="currentColor"
                    xmlns="http://www.w3.org/2000/svg"
                  >
                    <path d="M2 10L18 2L10 18L9 11L2 10Z" />
                  </svg>
                )}
              </button>
            </div>

            {/* Character count */}
            <div className={styles.inputFooter}>
              <span className={styles.charCount}>
                {input.length}/500
              </span>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
