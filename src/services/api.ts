/**
 * API client for communicating with FastAPI backend
 * Handles retries, timeouts, and error handling
 */

import type { ChatRequest, ChatResponse, HealthResponse, APIError } from '@site/src/types/api';

// Get API base URL from window config (set by api-config.js)
// Fallback to localhost for development
const getApiBaseUrl = (): string => {
  if (typeof window !== 'undefined' && (window as any).API_CONFIG) {
    return (window as any).API_CONFIG.baseUrl;
  }
  return 'http://localhost:8000';
};

const API_BASE_URL = getApiBaseUrl();
const DEFAULT_TIMEOUT = 30000; // 30 seconds
const MAX_RETRIES = 3;
const RETRY_DELAY = 1000; // 1 second

/**
 * Custom error class for API errors
 */
export class APIClientError extends Error {
  statusCode?: number;

  constructor(message: string, statusCode?: number) {
    super(message);
    this.name = 'APIClientError';
    this.statusCode = statusCode;
  }
}

/**
 * Sleep function for retry delays
 */
const sleep = (ms: number): Promise<void> => {
  return new Promise((resolve) => setTimeout(resolve, ms));
};

/**
 * Generic fetch with timeout
 */
async function fetchWithTimeout(
  url: string,
  options: RequestInit,
  timeout: number = DEFAULT_TIMEOUT
): Promise<Response> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeout);

  try {
    const response = await fetch(url, {
      ...options,
      signal: controller.signal,
    });
    clearTimeout(timeoutId);
    return response;
  } catch (error) {
    clearTimeout(timeoutId);
    if (error.name === 'AbortError') {
      throw new APIClientError('Request timeout', 408);
    }
    throw error;
  }
}

/**
 * Generic fetch with retries
 */
async function fetchWithRetries(
  url: string,
  options: RequestInit,
  retries: number = MAX_RETRIES
): Promise<Response> {
  let lastError: Error;

  for (let attempt = 0; attempt < retries; attempt++) {
    try {
      const response = await fetchWithTimeout(url, options);

      // Don't retry on client errors (4xx)
      if (response.status >= 400 && response.status < 500) {
        return response;
      }

      // Retry on server errors (5xx) or network errors
      if (response.ok || attempt === retries - 1) {
        return response;
      }

      // Wait before retrying (exponential backoff)
      await sleep(RETRY_DELAY * Math.pow(2, attempt));
    } catch (error) {
      lastError = error as Error;

      // Don't retry on timeout or abort
      if (error instanceof APIClientError) {
        throw error;
      }

      // Wait before retrying
      if (attempt < retries - 1) {
        await sleep(RETRY_DELAY * Math.pow(2, attempt));
      }
    }
  }

  throw lastError!;
}

/**
 * Send a chat message and get a response
 */
export async function chatWithRAG(
  query: string,
  conversationId?: string,
  topK: number = 8
): Promise<ChatResponse> {
  const request: ChatRequest = {
    query,
    conversation_id: conversationId,
    top_k: topK,
  };

  try {
    const response = await fetchWithRetries(`${API_BASE_URL}/api/v1/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      const errorData: APIError = await response.json().catch(() => ({
        detail: 'Failed to get response from server',
      }));
      throw new APIClientError(
        errorData.detail || `Request failed with status ${response.status}`,
        response.status
      );
    }

    return await response.json();
  } catch (error) {
    if (error instanceof APIClientError) {
      throw error;
    }
    throw new APIClientError(
      'Failed to connect to the server. Please check your connection and try again.',
      0
    );
  }
}

/**
 * Check the health status of the backend
 */
export async function checkHealth(): Promise<HealthResponse> {
  try {
    const response = await fetchWithTimeout(`${API_BASE_URL}/api/v1/health`, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new APIClientError(`Health check failed with status ${response.status}`, response.status);
    }

    return await response.json();
  } catch (error) {
    if (error instanceof APIClientError) {
      throw error;
    }
    throw new APIClientError('Failed to check health status', 0);
  }
}

/**
 * Generate a URL for a book section from a citation
 */
export function getCitationUrl(citation: {
  module: string;
  chapter: string;
  url_fragment: string;
}): string {
  // Remove "module-XX-" prefix if present to get the clean module name
  const cleanModule = citation.module.replace(/^module-\d+-/, '');

  // Construct the docs URL
  // Example: /docs/module-01-ros2/01-fundamentals#publisher-subscriber
  return `/docs/${citation.module}/${citation.chapter}${citation.url_fragment}`;
}
