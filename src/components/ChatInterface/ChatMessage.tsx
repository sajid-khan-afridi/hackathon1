/**
 * ChatMessage component for rendering individual messages
 * Handles user and assistant messages with different styling
 * Displays citations with clickable links to book sections
 */

import React from 'react';
import type { Message } from '@site/src/types/api';
import { getCitationUrl } from '@site/src/services/api';
import styles from './ChatInterface.module.css';

interface ChatMessageProps {
  message: Message;
}

export default function ChatMessage({ message }: ChatMessageProps): React.JSX.Element {
  const isUser = message.role === 'user';

  /**
   * Format timestamp to readable format
   */
  const formatTimestamp = (date: Date): string => {
    return date.toLocaleTimeString('en-US', {
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  /**
   * Render message content with basic markdown support
   * - Code blocks: ```language\ncode\n```
   * - Inline code: `code`
   * - Bold: **text**
   */
  const renderContent = (content: string): React.JSX.Element[] => {
    const parts: React.JSX.Element[] = [];
    let currentIndex = 0;

    // Code block regex: ```language\ncode\n```
    const codeBlockRegex = /```(\w+)?\n([\s\S]*?)```/g;
    let match: RegExpExecArray | null;

    while ((match = codeBlockRegex.exec(content)) !== null) {
      // Add text before code block
      if (match.index > currentIndex) {
        parts.push(
          <span key={`text-${currentIndex}`}>
            {content.slice(currentIndex, match.index)}
          </span>
        );
      }

      // Add code block
      const language = match[1] || 'text';
      const code = match[2];
      parts.push(
        <pre key={`code-${match.index}`} className={styles.codeBlock}>
          <code className={`language-${language}`}>{code}</code>
        </pre>
      );

      currentIndex = match.index + match[0].length;
    }

    // Add remaining text
    if (currentIndex < content.length) {
      parts.push(
        <span key={`text-${currentIndex}`}>
          {content.slice(currentIndex)}
        </span>
      );
    }

    return parts.length > 0 ? parts : [<span key="content">{content}</span>];
  };

  return (
    <div
      className={`${styles.message} ${
        isUser ? styles.userMessage : styles.assistantMessage
      }`}
    >
      {/* Message role indicator */}
      <div className={styles.messageRole}>
        {isUser ? 'You' : 'Physical AI Assistant'}
      </div>

      {/* Message content */}
      <div className={styles.messageContent}>{renderContent(message.content)}</div>

      {/* Citations (only for assistant messages) */}
      {!isUser && message.sources && message.sources.length > 0 && (
        <div className={styles.sources}>
          <div className={styles.sourcesHeader}>
            <strong>Sources:</strong>
          </div>
          <ul className={styles.sourcesList}>
            {message.sources.map((source, idx) => {
              const url = getCitationUrl(source);
              const displayText = `${source.module} → ${source.chapter}${
                source.section ? ` → ${source.section}` : ''
              }`;

              return (
                <li key={idx} className={styles.sourceItem}>
                  <a
                    href={url}
                    target="_blank"
                    rel="noopener noreferrer"
                    className={styles.sourceLink}
                    title={source.content_preview}
                  >
                    {displayText}
                  </a>
                  {source.relevance_score && (
                    <span className={styles.relevanceScore}>
                      {(source.relevance_score * 100).toFixed(0)}% relevant
                    </span>
                  )}
                </li>
              );
            })}
          </ul>
        </div>
      )}

      {/* Timestamp */}
      <div className={styles.timestamp}>{formatTimestamp(message.timestamp)}</div>
    </div>
  );
}
