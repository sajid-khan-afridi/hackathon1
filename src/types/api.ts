/**
 * TypeScript type definitions for API communication
 * Matches backend Pydantic schemas for type safety
 */

/**
 * Single message in a conversation
 */
export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Citation[];
  timestamp: Date;
}

/**
 * Citation linking to book content
 */
export interface Citation {
  module: string;
  chapter: string;
  section: string;
  url_fragment: string;
  content_preview?: string;
  relevance_score?: number;
}

/**
 * Request payload for chat endpoint
 */
export interface ChatRequest {
  query: string;
  conversation_id?: string;
  top_k?: number;
}

/**
 * Response from chat endpoint
 */
export interface ChatResponse {
  response: string;
  sources: Citation[];
  query_type: string;
  conversation_id: string;
  retrieval_time_ms: number;
  generation_time_ms: number;
  total_time_ms: number;
}

/**
 * Health check response
 */
export interface HealthResponse {
  status: 'healthy' | 'degraded' | 'unhealthy';
  services: Record<string, string>;
  version: string;
  timestamp: string;
}

/**
 * API error response
 */
export interface APIError {
  detail: string;
  status_code?: number;
}
