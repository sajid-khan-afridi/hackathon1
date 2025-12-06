"""Application configuration using Pydantic Settings"""

import os
from typing import List
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables

    All settings can be overridden via .env file or environment variables
    """

    # API Configuration
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Physical AI RAG API"
    VERSION: str = "0.1.0"
    DEBUG: bool = False
    ENVIRONMENT: str = os.getenv("ENVIRONMENT", "development")

    # CORS Configuration
    # Default includes localhost for development
    # In production, set ALLOWED_ORIGINS environment variable to include GitHub Pages URL
    ALLOWED_ORIGINS: str = "http://localhost:3000,http://localhost:8000,https://sajid-khan-afridi.github.io"

    @property
    def CORS_ORIGINS(self) -> List[str]:
        """Parse ALLOWED_ORIGINS as comma-separated list"""
        return [origin.strip() for origin in self.ALLOWED_ORIGINS.split(",")]

    # Database Configuration (Neon Postgres)
    DATABASE_URL: str = os.getenv(
        "DATABASE_URL",
        "postgresql://physical_ai_user:dev_password@localhost:5432/physical_ai"
    )

    # Database Connection Pooling
    DB_POOL_SIZE: int = int(os.getenv("DB_POOL_SIZE", "10"))
    DB_MAX_OVERFLOW: int = int(os.getenv("DB_MAX_OVERFLOW", "20"))
    DB_POOL_TIMEOUT: int = int(os.getenv("DB_POOL_TIMEOUT", "30"))
    DB_POOL_RECYCLE: int = 3600  # Recycle connections after 1 hour
    DB_ECHO: bool = False  # Set to True for SQL query logging

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_COLLECTION_NAME: str = "physical_ai_chunks"
    QDRANT_VECTOR_SIZE: int = 1536  # text-embedding-3-small dimension
    QDRANT_HNSW_M: int = 16  # Number of edges per node in HNSW graph
    QDRANT_HNSW_EF_CONSTRUCT: int = 100  # Size of dynamic candidate list

    # OpenAI Configuration
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")
    OPENAI_EMBEDDING_MODEL: str = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")
    OPENAI_EMBEDDING_DIMENSIONS: int = 1536
    OPENAI_CHAT_MODEL: str = os.getenv("CHAT_MODEL", "gpt-4-turbo-preview")
    OPENAI_MAX_RETRIES: int = 3
    OPENAI_TIMEOUT: int = 30

    # RAG Configuration
    CHUNK_SIZE_THEORETICAL: int = 800  # tokens
    CHUNK_SIZE_CODE: int = 1200  # tokens
    CHUNK_SIZE_MIXED: int = 1000  # tokens
    CHUNK_OVERLAP: int = 200  # tokens
    RETRIEVAL_TOP_K: int = int(os.getenv("TOP_K_CHUNKS", "20"))
    VECTOR_SEARCH_WEIGHT: float = float(os.getenv("VECTOR_WEIGHT", "0.7"))
    BM25_SEARCH_WEIGHT: float = float(os.getenv("BM25_WEIGHT", "0.3"))
    MAX_CONTEXT_TOKENS: int = int(os.getenv("MAX_CONTEXT_TOKENS", "5000"))

    # Performance Configuration
    MAX_QUERY_LENGTH: int = 500
    MAX_RESPONSE_TOKENS: int = 1000
    REQUEST_TIMEOUT: int = 30  # seconds
    EMBEDDING_CACHE_TTL: int = int(os.getenv("EMBEDDING_CACHE_TTL", "3600"))
    QUERY_CACHE_TTL: int = int(os.getenv("QUERY_CACHE_TTL", "300"))

    # Async Database URL (for SQLAlchemy async)
    @property
    def ASYNC_DATABASE_URL(self) -> str:
        """Convert postgres:// to postgresql+asyncpg:// for async operations"""
        if self.DATABASE_URL.startswith("postgresql://"):
            return self.DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
        return self.DATABASE_URL

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = True
        extra = "ignore"  # Ignore extra environment variables not defined in Settings


# Global settings instance
settings = Settings()
