"""Tests for RAG service"""

import pytest
from app.services.rag import RAGService


@pytest.fixture
def rag_service():
    """RAG service fixture"""
    return RAGService()


def test_rag_service_initialization(rag_service):
    """Test RAG service can be initialized"""
    assert rag_service is not None


@pytest.mark.asyncio
async def test_generate_response_structure(rag_service, sample_query):
    """Test that generated response has correct structure"""
    response = await rag_service.generate_response(sample_query)

    assert hasattr(response, "response")
    assert hasattr(response, "sources")
    assert isinstance(response.sources, list)


@pytest.mark.asyncio
async def test_generate_response_includes_citations(rag_service, sample_query):
    """Test that response includes citations"""
    response = await rag_service.generate_response(sample_query)

    # Should have at least one citation
    # TODO: Verify once RAG pipeline is implemented
    pass
