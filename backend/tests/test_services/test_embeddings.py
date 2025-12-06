"""Comprehensive tests for embeddings service"""

import pytest
from unittest.mock import Mock, AsyncMock, patch, MagicMock
from app.services.embeddings import EmbeddingsService
from openai import RateLimitError, APIError


@pytest.fixture
def embeddings_service():
    """Embeddings service fixture"""
    return EmbeddingsService()


@pytest.fixture
def mock_openai_response():
    """Mock OpenAI embedding response"""
    mock_response = Mock()
    mock_response.data = [Mock()]
    mock_response.data[0].embedding = [0.1] * 1536
    return mock_response


class TestEmbeddingsServiceInit:
    """Test embeddings service initialization"""

    def test_service_initialization(self, embeddings_service):
        """Test embeddings service can be initialized"""
        assert embeddings_service is not None
        assert embeddings_service.model == "text-embedding-3-small"
        assert embeddings_service.dimensions == 1536
        assert embeddings_service.client is not None
        assert isinstance(embeddings_service._cache, dict)
        assert embeddings_service._cache_hits == 0
        assert embeddings_service._cache_misses == 0


class TestSingleEmbeddingGeneration:
    """Test single embedding generation"""

    @pytest.mark.asyncio
    async def test_generate_embedding_success(self, embeddings_service, mock_openai_response):
        """Test successful embedding generation"""
        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_openai_response
        ):
            text = "What is ROS2?"
            embedding = await embeddings_service.generate_embedding(text)

            assert embedding is not None
            assert len(embedding) == 1536
            assert all(isinstance(x, (int, float)) for x in embedding)

    @pytest.mark.asyncio
    async def test_generate_embedding_empty_text(self, embeddings_service):
        """Test embedding generation with empty text returns zero vector"""
        embedding = await embeddings_service.generate_embedding("")

        assert embedding is not None
        assert len(embedding) == 1536
        assert all(x == 0.0 for x in embedding)

    @pytest.mark.asyncio
    async def test_generate_embedding_whitespace(self, embeddings_service):
        """Test embedding generation with whitespace returns zero vector"""
        embedding = await embeddings_service.generate_embedding("   \n\t  ")

        assert embedding is not None
        assert len(embedding) == 1536
        assert all(x == 0.0 for x in embedding)

    @pytest.mark.asyncio
    async def test_generate_embedding_cache_disabled(self, embeddings_service, mock_openai_response):
        """Test embedding generation with cache disabled"""
        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_openai_response
        ):
            text = "What is ROS2?"

            # First call with cache disabled
            embedding1 = await embeddings_service.generate_embedding(text, use_cache=False)

            # Second call with cache disabled
            embedding2 = await embeddings_service.generate_embedding(text, use_cache=False)

            assert embedding1 == embedding2
            # Cache should not be populated
            assert len(embeddings_service._cache) == 0


class TestEmbeddingCaching:
    """Test embedding caching functionality"""

    @pytest.mark.asyncio
    async def test_cache_hit(self, embeddings_service, mock_openai_response):
        """Test that cache is used on second call"""
        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_openai_response
        ) as mock_create:
            text = "What is ROS2?"

            # Clear cache and stats
            embeddings_service.clear_cache()

            # First call - cache miss
            embedding1 = await embeddings_service.generate_embedding(text)
            assert mock_create.call_count == 1
            assert embeddings_service._cache_misses == 1
            assert embeddings_service._cache_hits == 0

            # Second call - cache hit
            embedding2 = await embeddings_service.generate_embedding(text)
            assert mock_create.call_count == 1  # No additional API call
            assert embeddings_service._cache_misses == 1
            assert embeddings_service._cache_hits == 1

            # Embeddings should be identical
            assert embedding1 == embedding2

    @pytest.mark.asyncio
    async def test_cache_eviction(self, embeddings_service, mock_openai_response):
        """Test cache eviction when max size is reached"""
        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_openai_response
        ):
            embeddings_service.clear_cache()

            # Fill cache to capacity (1000 entries)
            for i in range(1001):
                text = f"Text {i}"
                await embeddings_service.generate_embedding(text)

            # Cache should be at max size (oldest entry evicted)
            assert len(embeddings_service._cache) == 1000

    def test_cache_key_generation(self, embeddings_service):
        """Test cache key generation"""
        text1 = "What is ROS2?"
        text2 = "What is ROS2?"
        text3 = "Different text"

        key1 = embeddings_service._get_cache_key(text1)
        key2 = embeddings_service._get_cache_key(text2)
        key3 = embeddings_service._get_cache_key(text3)

        # Same text should produce same key
        assert key1 == key2
        # Different text should produce different key
        assert key1 != key3

    def test_cache_stats(self, embeddings_service):
        """Test cache statistics"""
        embeddings_service.clear_cache()
        embeddings_service._cache_hits = 10
        embeddings_service._cache_misses = 5

        stats = embeddings_service.get_cache_stats()

        assert stats['cache_hits'] == 10
        assert stats['cache_misses'] == 5
        assert stats['total_requests'] == 15
        assert stats['hit_rate'] == 10 / 15
        assert 'cache_size' in stats

    def test_clear_cache(self, embeddings_service):
        """Test cache clearing"""
        # Populate cache
        embeddings_service._cache['key1'] = [0.1] * 1536
        embeddings_service._cache['key2'] = [0.2] * 1536
        embeddings_service._cache_hits = 5
        embeddings_service._cache_misses = 3

        # Clear cache
        embeddings_service.clear_cache()

        assert len(embeddings_service._cache) == 0
        assert embeddings_service._cache_hits == 0
        assert embeddings_service._cache_misses == 0


class TestBatchEmbeddingGeneration:
    """Test batch embedding generation"""

    @pytest.mark.asyncio
    async def test_batch_generate_embeddings_success(self, embeddings_service):
        """Test successful batch embedding generation"""
        mock_response = Mock()
        mock_response.data = [Mock() for _ in range(3)]
        for i, data in enumerate(mock_response.data):
            data.embedding = [0.1 + i * 0.1] * 1536

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            texts = ["Text 1", "Text 2", "Text 3"]
            embeddings = await embeddings_service.batch_generate_embeddings(texts)

            assert len(embeddings) == 3
            assert all(len(emb) == 1536 for emb in embeddings)

    @pytest.mark.asyncio
    async def test_batch_generate_empty_list(self, embeddings_service):
        """Test batch generation with empty list"""
        embeddings = await embeddings_service.batch_generate_embeddings([])
        assert embeddings == []

    @pytest.mark.asyncio
    async def test_batch_generate_with_cache(self, embeddings_service):
        """Test batch generation uses cache"""
        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ) as mock_create:
            embeddings_service.clear_cache()

            # First batch
            texts = ["Text 1", "Text 2"]
            embeddings1 = await embeddings_service.batch_generate_embeddings(texts)

            # Second batch with same texts - should use cache
            embeddings2 = await embeddings_service.batch_generate_embeddings(texts)

            # Should only call API once (for first batch)
            # Second batch should be fully cached
            assert embeddings1 == embeddings2
            assert embeddings_service._cache_hits > 0

    @pytest.mark.asyncio
    async def test_batch_generate_with_empty_texts(self, embeddings_service):
        """Test batch generation handles empty texts"""
        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            texts = ["Valid text", "", "  ", "Another valid"]
            embeddings = await embeddings_service.batch_generate_embeddings(texts)

            assert len(embeddings) == 4
            # Empty texts should have zero vectors
            assert all(x == 0.0 for x in embeddings[1])
            assert all(x == 0.0 for x in embeddings[2])

    @pytest.mark.asyncio
    async def test_batch_generate_large_batch(self, embeddings_service):
        """Test batch generation with large batch (batching logic)"""
        mock_response = Mock()
        mock_response.data = [Mock() for _ in range(50)]
        for data in mock_response.data:
            data.embedding = [0.1] * 1536

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ) as mock_create:
            embeddings_service.clear_cache()

            # Generate 150 embeddings (should be split into 2 batches of 100 and 50)
            texts = [f"Text {i}" for i in range(150)]
            embeddings = await embeddings_service.batch_generate_embeddings(
                texts,
                batch_size=100
            )

            assert len(embeddings) == 150
            # Should make 2 API calls (2 batches)
            assert mock_create.call_count == 2


class TestErrorHandling:
    """Test error handling and retry logic"""

    @pytest.mark.asyncio
    async def test_rate_limit_retry(self, embeddings_service):
        """Test retry logic on rate limit error"""
        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            side_effect=[
                RateLimitError("Rate limit exceeded"),
                mock_response  # Success on retry
            ]
        ):
            text = "What is ROS2?"
            embedding = await embeddings_service.generate_embedding(text, use_cache=False)

            assert embedding is not None
            assert len(embedding) == 1536

    @pytest.mark.asyncio
    async def test_api_error_retry(self, embeddings_service):
        """Test retry logic on API error"""
        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            side_effect=[
                APIError("API error"),
                mock_response  # Success on retry
            ]
        ):
            text = "What is ROS2?"
            embedding = await embeddings_service.generate_embedding(text, use_cache=False)

            assert embedding is not None
            assert len(embedding) == 1536

    @pytest.mark.asyncio
    async def test_max_retries_exceeded(self, embeddings_service):
        """Test that exception is raised after max retries"""
        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            side_effect=RateLimitError("Rate limit exceeded")
        ):
            text = "What is ROS2?"

            with pytest.raises(RateLimitError):
                await embeddings_service.generate_embedding(text, use_cache=False)


class TestHealthCheck:
    """Test OpenAI health check"""

    @pytest.mark.asyncio
    async def test_health_check_success(self, embeddings_service, mock_openai_response):
        """Test successful health check"""
        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_openai_response
        ):
            is_healthy = await embeddings_service.check_openai_health()
            assert is_healthy is True

    @pytest.mark.asyncio
    async def test_health_check_failure(self, embeddings_service):
        """Test failed health check"""
        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            side_effect=Exception("Connection failed")
        ):
            is_healthy = await embeddings_service.check_openai_health()
            assert is_healthy is False

    @pytest.mark.asyncio
    async def test_health_check_empty_response(self, embeddings_service):
        """Test health check with empty response"""
        mock_response = Mock()
        mock_response.data = []

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            is_healthy = await embeddings_service.check_openai_health()
            assert is_healthy is False
