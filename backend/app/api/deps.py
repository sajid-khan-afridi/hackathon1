"""Dependency injection for API routes"""

from app.services.rag import RAGService
from app.services.storage import StorageService
from app.services.retrieval import RetrievalService
from app.services.embeddings import EmbeddingsService
from app.core.config import settings


async def get_storage_service() -> StorageService:
    """
    Get StorageService instance

    Returns:
        StorageService for database and Qdrant operations
    """
    # TODO: Implement proper dependency injection with connection pooling
    return StorageService()


async def get_retrieval_service() -> RetrievalService:
    """
    Get RetrievalService instance

    Returns:
        RetrievalService for hybrid search
    """
    # TODO: Implement proper dependency injection
    return RetrievalService()


async def get_embeddings_service() -> EmbeddingsService:
    """
    Get EmbeddingsService instance

    Returns:
        EmbeddingsService for generating embeddings and OpenAI health checks
    """
    # TODO: Implement proper dependency injection
    return EmbeddingsService()


async def get_rag_service() -> RAGService:
    """
    Get RAGService instance

    Returns:
        RAGService for complete RAG pipeline
    """
    # TODO: Implement proper dependency injection
    return RAGService()
