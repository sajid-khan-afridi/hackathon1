"""Comprehensive tests for retrieval service"""

import pytest
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from app.services.retrieval import RetrievalService


@pytest.fixture
def retrieval_service():
    """Retrieval service fixture"""
    return RetrievalService()


@pytest.fixture
def mock_query_embedding():
    """Mock query embedding"""
    return [0.1] * 1536


@pytest.fixture
def mock_vector_results():
    """Mock vector search results"""
    return [
        {
            "vector_id": "doc1_0",
            "score": 0.95,
            "payload": {
                "chunk_id": "chunk-1",
                "module": "module-01-ros2",
                "chapter": "01-fundamentals",
                "section": "Introduction",
                "chunk_type": "theoretical",
                "learning_level": "beginner"
            }
        },
        {
            "vector_id": "doc1_1",
            "score": 0.85,
            "payload": {
                "chunk_id": "chunk-2",
                "module": "module-01-ros2",
                "chapter": "02-architecture",
                "section": "Communication",
                "chunk_type": "mixed",
                "learning_level": "intermediate"
            }
        }
    ]


@pytest.fixture
def mock_bm25_results():
    """Mock BM25 search results"""
    return [
        {
            "chunk_id": "chunk-1",
            "content": "ROS2 is the second generation of Robot Operating System...",
            "module": "module-01-ros2",
            "chapter": "01-fundamentals",
            "section": "Introduction",
            "chunk_type": "theoretical",
            "bm25_score": 0.75
        },
        {
            "chunk_id": "chunk-3",
            "content": "Publisher nodes send messages to topics...",
            "module": "module-01-ros2",
            "chapter": "03-communication",
            "section": "Publishers",
            "chunk_type": "code",
            "bm25_score": 0.65
        }
    ]


class TestRetrievalServiceInit:
    """Test retrieval service initialization"""

    def test_service_initialization(self, retrieval_service):
        """Test retrieval service can be initialized"""
        assert retrieval_service is not None
        assert retrieval_service.vector_weight == 0.7
        assert retrieval_service.bm25_weight == 0.3
        assert retrieval_service.openai_client is not None


class TestQueryClassification:
    """Test query type classification"""

    @pytest.mark.asyncio
    async def test_conceptual_query_classification(self, retrieval_service):
        """Test classification of conceptual queries"""
        queries = [
            "What is ROS2?",
            "Explain robot kinematics",
            "Define forward kinematics",
            "Tell me about Gazebo",
            "Describe Unity for robotics"
        ]

        for query in queries:
            query_type = await retrieval_service.classify_query(query)
            assert query_type == "conceptual", f"Failed for query: {query}"

    @pytest.mark.asyncio
    async def test_code_query_classification(self, retrieval_service):
        """Test classification of code queries"""
        queries = [
            "Show me a ROS2 publisher example",
            "How to create a subscriber?",
            "Implement a service client",
            "Write a launch file",
            "Sample code for action server"
        ]

        for query in queries:
            query_type = await retrieval_service.classify_query(query)
            assert query_type == "code", f"Failed for query: {query}"

    @pytest.mark.asyncio
    async def test_troubleshooting_query_classification(self, retrieval_service):
        """Test classification of troubleshooting queries"""
        queries = [
            "How to fix node not found error?",
            "Debug topic not publishing",
            "Error: package not found",
            "My robot won't move",
            "Can't connect to Gazebo"
        ]

        for query in queries:
            query_type = await retrieval_service.classify_query(query)
            assert query_type == "troubleshooting", f"Failed for query: {query}"

    @pytest.mark.asyncio
    async def test_comparison_query_classification(self, retrieval_service):
        """Test classification of comparison queries"""
        queries = [
            "Difference between Gazebo and Unity?",
            "ROS1 vs ROS2",
            "Which is better: URDF or SDF?",
            "Compare forward and inverse kinematics",
            "When to use actions vs services?"
        ]

        for query in queries:
            query_type = await retrieval_service.classify_query(query)
            assert query_type == "comparison", f"Failed for query: {query}"

    @pytest.mark.asyncio
    async def test_default_classification(self, retrieval_service):
        """Test default classification for ambiguous queries"""
        query = "robot"
        query_type = await retrieval_service.classify_query(query)
        assert query_type == "conceptual"  # Default


class TestSearchWeights:
    """Test adaptive search weights based on query type"""

    def test_conceptual_weights(self, retrieval_service):
        """Test weights for conceptual queries"""
        weights = retrieval_service.get_search_weights("conceptual")
        assert weights["vector"] == 0.8
        assert weights["bm25"] == 0.2

    def test_code_weights(self, retrieval_service):
        """Test weights for code queries"""
        weights = retrieval_service.get_search_weights("code")
        assert weights["vector"] == 0.5
        assert weights["bm25"] == 0.5

    def test_troubleshooting_weights(self, retrieval_service):
        """Test weights for troubleshooting queries"""
        weights = retrieval_service.get_search_weights("troubleshooting")
        assert weights["vector"] == 0.6
        assert weights["bm25"] == 0.4

    def test_comparison_weights(self, retrieval_service):
        """Test weights for comparison queries"""
        weights = retrieval_service.get_search_weights("comparison")
        assert weights["vector"] == 0.7
        assert weights["bm25"] == 0.3

    def test_default_weights(self, retrieval_service):
        """Test default weights for unknown query type"""
        weights = retrieval_service.get_search_weights("unknown")
        assert weights["vector"] == 0.7
        assert weights["bm25"] == 0.3


class TestHybridSearch:
    """Test hybrid search functionality"""

    @pytest.mark.asyncio
    async def test_hybrid_search_success(
        self,
        retrieval_service,
        mock_query_embedding,
        mock_vector_results,
        mock_bm25_results
    ):
        """Test successful hybrid search"""
        # Mock embedding generation
        with patch.object(
            retrieval_service,
            'generate_query_embedding',
            new_callable=AsyncMock,
            return_value=mock_query_embedding
        ):
            # Mock vector search
            with patch('app.services.storage.storage_service.search_vectors', new_callable=AsyncMock) as mock_vector_search:
                mock_vector_search.return_value = mock_vector_results

                # Mock BM25 search
                with patch.object(
                    retrieval_service,
                    'bm25_search',
                    new_callable=AsyncMock,
                    return_value=mock_bm25_results
                ):
                    # Mock enrich results
                    with patch.object(
                        retrieval_service,
                        'enrich_results',
                        new_callable=AsyncMock,
                        return_value=[
                            {
                                "chunk_id": "chunk-1",
                                "content": "Test content",
                                "module": "module-01-ros2",
                                "chapter": "01-fundamentals",
                                "vector_score": 0.95,
                                "bm25_score": 0.75,
                                "hybrid_score": 0.89,
                                "chunk_type": "theoretical"
                            }
                        ]
                    ):
                        results = await retrieval_service.hybrid_search(
                            query="What is ROS2?",
                            top_k=5,
                            query_type="conceptual"
                        )

                        assert len(results) <= 5
                        assert all('hybrid_score' in r for r in results)

    @pytest.mark.asyncio
    async def test_hybrid_search_with_filters(
        self,
        retrieval_service,
        mock_query_embedding
    ):
        """Test hybrid search with metadata filters"""
        filters = {
            "module": "module-01-ros2",
            "chunk_type": "code",
            "learning_level": "beginner"
        }

        with patch.object(
            retrieval_service,
            'generate_query_embedding',
            new_callable=AsyncMock,
            return_value=mock_query_embedding
        ):
            with patch('app.services.storage.storage_service.search_vectors', new_callable=AsyncMock) as mock_vector_search:
                mock_vector_search.return_value = []

                with patch.object(
                    retrieval_service,
                    'bm25_search',
                    new_callable=AsyncMock,
                    return_value=[]
                ):
                    with patch.object(
                        retrieval_service,
                        'enrich_results',
                        new_callable=AsyncMock,
                        return_value=[]
                    ):
                        results = await retrieval_service.hybrid_search(
                            query="Show me ROS2 code",
                            top_k=5,
                            filters=filters
                        )

                        # Verify filters were passed to searches
                        mock_vector_search.assert_called_once()
                        call_kwargs = mock_vector_search.call_args[1]
                        assert call_kwargs["module"] == "module-01-ros2"
                        assert call_kwargs["chunk_type"] == "code"
                        assert call_kwargs["learning_level"] == "beginner"

    @pytest.mark.asyncio
    async def test_hybrid_search_empty_results(
        self,
        retrieval_service,
        mock_query_embedding
    ):
        """Test hybrid search with no results"""
        with patch.object(
            retrieval_service,
            'generate_query_embedding',
            new_callable=AsyncMock,
            return_value=mock_query_embedding
        ):
            with patch('app.services.storage.storage_service.search_vectors', new_callable=AsyncMock) as mock_vector_search:
                mock_vector_search.return_value = []

                with patch.object(
                    retrieval_service,
                    'bm25_search',
                    new_callable=AsyncMock,
                    return_value=[]
                ):
                    with patch.object(
                        retrieval_service,
                        'enrich_results',
                        new_callable=AsyncMock,
                        return_value=[]
                    ):
                        results = await retrieval_service.hybrid_search(
                            query="Nonexistent topic",
                            top_k=5
                        )

                        assert results == []


class TestReranking:
    """Test result reranking based on query type"""

    def test_rerank_code_query(self, retrieval_service):
        """Test reranking for code queries boosts code chunks"""
        results = [
            {
                "chunk_id": "1",
                "chunk_type": "code",
                "hybrid_score": 0.7,
                "content": "Code chunk"
            },
            {
                "chunk_id": "2",
                "chunk_type": "theoretical",
                "hybrid_score": 0.8,
                "content": "Theory chunk"
            },
            {
                "chunk_id": "3",
                "chunk_type": "mixed",
                "hybrid_score": 0.75,
                "content": "Mixed chunk"
            }
        ]

        reranked = retrieval_service.rerank_results(results, "code")

        # Code chunks should get boost
        code_chunk = next(r for r in reranked if r["chunk_type"] == "code")
        assert code_chunk["rerank_boost"] == 0.15
        assert code_chunk["hybrid_score"] == 0.7 + 0.15

    def test_rerank_conceptual_query(self, retrieval_service):
        """Test reranking for conceptual queries boosts theoretical chunks"""
        results = [
            {
                "chunk_id": "1",
                "chunk_type": "code",
                "hybrid_score": 0.8,
                "content": "Code chunk"
            },
            {
                "chunk_id": "2",
                "chunk_type": "theoretical",
                "hybrid_score": 0.7,
                "content": "Theory chunk"
            }
        ]

        reranked = retrieval_service.rerank_results(results, "conceptual")

        # Theoretical chunks should get boost
        theory_chunk = next(r for r in reranked if r["chunk_type"] == "theoretical")
        assert theory_chunk["rerank_boost"] == 0.1

    def test_rerank_comparison_query(self, retrieval_service):
        """Test reranking for comparison queries boosts mixed chunks"""
        results = [
            {
                "chunk_id": "1",
                "chunk_type": "mixed",
                "hybrid_score": 0.7,
                "content": "Mixed chunk"
            },
            {
                "chunk_id": "2",
                "chunk_type": "code",
                "hybrid_score": 0.8,
                "content": "Code chunk"
            }
        ]

        reranked = retrieval_service.rerank_results(results, "comparison")

        # Mixed chunks should get boost
        mixed_chunk = next(r for r in reranked if r["chunk_type"] == "mixed")
        assert mixed_chunk["rerank_boost"] == 0.1

    def test_rerank_troubleshooting_query(self, retrieval_service):
        """Test reranking for troubleshooting queries boosts code/mixed chunks"""
        results = [
            {
                "chunk_id": "1",
                "chunk_type": "code",
                "hybrid_score": 0.7,
                "content": "Code chunk"
            },
            {
                "chunk_id": "2",
                "chunk_type": "theoretical",
                "hybrid_score": 0.8,
                "content": "Theory chunk"
            }
        ]

        reranked = retrieval_service.rerank_results(results, "troubleshooting")

        # Code chunks should get boost for troubleshooting
        code_chunk = next(r for r in reranked if r["chunk_type"] == "code")
        assert code_chunk["rerank_boost"] == 0.12

    def test_rerank_sorting(self, retrieval_service):
        """Test that reranking sorts by boosted scores"""
        results = [
            {"chunk_id": "1", "chunk_type": "code", "hybrid_score": 0.6},
            {"chunk_id": "2", "chunk_type": "theoretical", "hybrid_score": 0.8},
            {"chunk_id": "3", "chunk_type": "code", "hybrid_score": 0.7}
        ]

        reranked = retrieval_service.rerank_results(results, "code")

        # Results should be sorted by boosted score
        scores = [r["hybrid_score"] for r in reranked]
        assert scores == sorted(scores, reverse=True)


class TestGenerateQueryEmbedding:
    """Test query embedding generation"""

    @pytest.mark.asyncio
    async def test_generate_query_embedding_success(self, retrieval_service):
        """Test successful query embedding generation"""
        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        with patch.object(
            retrieval_service.openai_client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            embedding = await retrieval_service.generate_query_embedding("What is ROS2?")

            assert embedding is not None
            assert len(embedding) == 1536

    @pytest.mark.asyncio
    async def test_generate_query_embedding_failure(self, retrieval_service):
        """Test query embedding generation failure"""
        with patch.object(
            retrieval_service.openai_client.embeddings,
            'create',
            new_callable=AsyncMock,
            side_effect=Exception("API error")
        ):
            with pytest.raises(Exception):
                await retrieval_service.generate_query_embedding("What is ROS2?")


class TestBM25Search:
    """Test BM25 full-text search"""

    @pytest.mark.asyncio
    async def test_bm25_search_success(self, retrieval_service, mock_bm25_results):
        """Test successful BM25 search"""
        mock_session = MagicMock()
        mock_result = MagicMock()
        mock_result.fetchall.return_value = [
            ("chunk-1", "ROS2 content", "module-01-ros2", "01-fundamentals", "Intro", "theoretical", 0.75)
        ]
        mock_session.execute = AsyncMock(return_value=mock_result)

        with patch('app.services.storage.storage_service.get_session') as mock_get_session:
            mock_get_session.return_value.__aenter__.return_value = mock_session

            results = await retrieval_service.bm25_search("ROS2", limit=10)

            assert len(results) == 1
            assert results[0]["chunk_id"] == "chunk-1"
            assert "bm25_score" in results[0]

    @pytest.mark.asyncio
    async def test_bm25_search_with_filters(self, retrieval_service):
        """Test BM25 search with metadata filters"""
        mock_session = MagicMock()
        mock_result = MagicMock()
        mock_result.fetchall.return_value = []
        mock_session.execute = AsyncMock(return_value=mock_result)

        with patch('app.services.storage.storage_service.get_session') as mock_get_session:
            mock_get_session.return_value.__aenter__.return_value = mock_session

            filters = {
                "module": "module-01-ros2",
                "chunk_type": "code"
            }

            results = await retrieval_service.bm25_search("ROS2", filters=filters)

            # Verify SQL query was executed with filters
            assert mock_session.execute.called

    @pytest.mark.asyncio
    async def test_bm25_search_error_handling(self, retrieval_service):
        """Test BM25 search error handling"""
        with patch('app.services.storage.storage_service.get_session') as mock_get_session:
            mock_get_session.side_effect = Exception("Database error")

            # Should return empty list on error
            results = await retrieval_service.bm25_search("ROS2")
            assert results == []
