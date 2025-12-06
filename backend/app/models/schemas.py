"""Pydantic schemas for request/response validation"""

from typing import List, Optional, Dict, Any
from pydantic import BaseModel, Field, validator
from datetime import datetime
from uuid import UUID


# Chat API Schemas
class ChatRequest(BaseModel):
    """Request schema for chat endpoint"""
    query: str = Field(..., min_length=1, max_length=500, description="User question")
    conversation_id: Optional[str] = Field(None, description="Conversation ID for context continuity")
    top_k: int = Field(default=8, ge=1, le=20, description="Number of chunks to retrieve")

    @validator("query")
    def validate_query(cls, v):
        """Ensure query is not just whitespace"""
        if not v.strip():
            raise ValueError("Query cannot be empty or whitespace only")
        return v.strip()


class Citation(BaseModel):
    """Citation schema linking to book content"""
    module: str = Field(..., description="Module name (e.g., 'module-01-ros2')")
    chapter: str = Field(..., description="Chapter name")
    section: str = Field(..., description="Section name")
    url_fragment: str = Field(..., description="URL fragment for linking (e.g., '#introduction')")
    content_preview: Optional[str] = Field(None, description="Preview of the cited content")
    relevance_score: Optional[float] = Field(None, description="Relevance score (0-1)")


class ChatResponse(BaseModel):
    """Response schema for chat endpoint"""
    response: str = Field(..., description="Generated answer from RAG pipeline")
    sources: List[Citation] = Field(default_factory=list, description="Citations to book content")
    query_type: str = Field(default="general", description="Type of query (factual, procedural, conceptual)")
    conversation_id: str = Field(..., description="Conversation ID for tracking")
    retrieval_time_ms: int = Field(default=0, description="Time taken for retrieval in milliseconds")
    generation_time_ms: int = Field(default=0, description="Time taken for generation in milliseconds")
    total_time_ms: int = Field(default=0, description="Total time taken in milliseconds")


# Search API Schemas
class SearchRequest(BaseModel):
    """Request schema for search endpoint"""
    query: str = Field(..., min_length=1, max_length=500, description="Search query")
    top_k: int = Field(default=8, ge=1, le=20, description="Number of results to return")
    filters: Optional[Dict[str, Any]] = Field(default=None, description="Metadata filters")


class RetrievalResult(BaseModel):
    """Single retrieval result from hybrid search"""
    chunk_id: UUID
    content: str
    score: float
    metadata: Dict[str, Any]


class SearchResponse(BaseModel):
    """Response schema for search endpoint"""
    results: List[RetrievalResult] = Field(default_factory=list, description="Retrieved chunks")


# Document API Schemas
class DocumentResponse(BaseModel):
    """Response schema for document listing"""
    id: UUID
    title: str
    module: str
    chapter: str
    section: Optional[str] = None
    content_type: str
    file_path: str
    metadata: Dict[str, Any] = Field(default_factory=dict)
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True  # Enable ORM mode for SQLAlchemy models


# Health API Schemas
class HealthResponse(BaseModel):
    """Response schema for health check endpoint"""
    status: str = Field(..., description="Overall status: 'healthy', 'degraded', or 'unhealthy'")
    services: Dict[str, str] = Field(..., description="Individual service statuses")
    version: str = Field(default="0.1.0", description="API version")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Check timestamp")


# Ingestion Schemas
class ChunkCreate(BaseModel):
    """Schema for creating a new chunk"""
    document_id: UUID
    content: str
    chunk_index: int
    token_count: int
    vector_id: str
    chunk_type: str
    learning_level: Optional[str] = None
    metadata: Dict[str, Any] = Field(default_factory=dict)


class DocumentCreate(BaseModel):
    """Schema for creating a new document"""
    title: str
    module: str
    chapter: str
    section: Optional[str] = None
    content_type: str
    file_path: str
    metadata: Dict[str, Any] = Field(default_factory=dict)
