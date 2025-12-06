"""Performance tests for Physical AI RAG system"""

import pytest
import time
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from statistics import mean, median
from typing import List


@pytest.mark.performance
class TestEmbeddingPerformance:
    """Test embedding generation performance"""

    @pytest.mark.asyncio
    async def test_single_embedding_latency(self):
        """Test single embedding generation latency"""
        from app.services.embeddings import embeddings_service

        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        latencies = []

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            # Measure 10 embedding generations
            for i in range(10):
                start = time.time()
                await embeddings_service.generate_embedding(f"Test text {i}", use_cache=False)
                latency_ms = (time.time() - start) * 1000
                latencies.append(latency_ms)

        # Performance assertions
        p50 = median(latencies)
        p95 = sorted(latencies)[int(len(latencies) * 0.95)]
        avg = mean(latencies)

        print(f"\nSingle Embedding Latency:")
        print(f"  P50: {p50:.2f}ms")
        print(f"  P95: {p95:.2f}ms")
        print(f"  Avg: {avg:.2f}ms")

        # These should be very fast with mocking (< 50ms)
        assert p50 < 50, f"P50 latency too high: {p50}ms"
        assert p95 < 100, f"P95 latency too high: {p95}ms"

    @pytest.mark.asyncio
    async def test_batch_embedding_performance(self):
        """Test batch embedding generation performance"""
        from app.services.embeddings import embeddings_service

        mock_response = Mock()
        mock_response.data = [Mock() for _ in range(50)]
        for data in mock_response.data:
            data.embedding = [0.1] * 1536

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            texts = [f"Text {i}" for i in range(50)]

            start = time.time()
            await embeddings_service.batch_generate_embeddings(texts, use_cache=False)
            elapsed_ms = (time.time() - start) * 1000

            per_embedding_ms = elapsed_ms / 50

            print(f"\nBatch Embedding Performance (50 embeddings):")
            print(f"  Total: {elapsed_ms:.2f}ms")
            print(f"  Per embedding: {per_embedding_ms:.2f}ms")

            # Batch should be more efficient than individual calls
            assert per_embedding_ms < 10, f"Batch performance too slow: {per_embedding_ms}ms per embedding"

    @pytest.mark.asyncio
    async def test_cache_performance(self):
        """Test cache hit performance"""
        from app.services.embeddings import embeddings_service

        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        embeddings_service.clear_cache()

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            text = "Cache test text"

            # First call - cache miss
            start = time.time()
            await embeddings_service.generate_embedding(text)
            miss_time_ms = (time.time() - start) * 1000

            # Second call - cache hit
            start = time.time()
            await embeddings_service.generate_embedding(text)
            hit_time_ms = (time.time() - start) * 1000

            print(f"\nCache Performance:")
            print(f"  Cache miss: {miss_time_ms:.2f}ms")
            print(f"  Cache hit: {hit_time_ms:.2f}ms")
            print(f"  Speedup: {miss_time_ms/hit_time_ms:.1f}x")

            # Cache hits should be much faster
            assert hit_time_ms < miss_time_ms, "Cache hit should be faster than miss"
            assert hit_time_ms < 1.0, f"Cache hit too slow: {hit_time_ms}ms"


@pytest.mark.performance
class TestRetrievalPerformance:
    """Test retrieval performance"""

    @pytest.mark.asyncio
    async def test_query_classification_speed(self):
        """Test query classification speed"""
        from app.services.retrieval import retrieval_service

        queries = [
            "What is ROS2?",
            "Show me code",
            "How to fix error?",
            "Compare X vs Y",
            "Explain this concept"
        ] * 10  # 50 queries total

        start = time.time()
        for query in queries:
            await retrieval_service.classify_query(query)
        elapsed_ms = (time.time() - start) * 1000

        per_query_ms = elapsed_ms / len(queries)

        print(f"\nQuery Classification Performance ({len(queries)} queries):")
        print(f"  Total: {elapsed_ms:.2f}ms")
        print(f"  Per query: {per_query_ms:.2f}ms")

        # Classification should be very fast (pure pattern matching)
        assert per_query_ms < 1.0, f"Classification too slow: {per_query_ms}ms per query"

    @pytest.mark.asyncio
    async def test_hybrid_search_latency(self):
        """Test end-to-end hybrid search latency"""
        from app.services.retrieval import retrieval_service

        mock_embedding = [0.1] * 1536
        mock_vector_results = []
        mock_bm25_results = []

        latencies = []

        with patch.object(
            retrieval_service,
            'generate_query_embedding',
            new_callable=AsyncMock,
            return_value=mock_embedding
        ):
            with patch('app.services.storage.storage_service.search_vectors', new_callable=AsyncMock) as mock_vec:
                mock_vec.return_value = mock_vector_results

                with patch.object(retrieval_service, 'bm25_search', new_callable=AsyncMock) as mock_bm25:
                    mock_bm25.return_value = mock_bm25_results

                    with patch.object(retrieval_service, 'enrich_results', new_callable=AsyncMock) as mock_enrich:
                        mock_enrich.return_value = []

                        # Measure 10 searches
                        for i in range(10):
                            start = time.time()
                            await retrieval_service.hybrid_search(f"Query {i}")
                            latency_ms = (time.time() - start) * 1000
                            latencies.append(latency_ms)

        p50 = median(latencies)
        p95 = sorted(latencies)[int(len(latencies) * 0.95)]

        print(f"\nHybrid Search Latency:")
        print(f"  P50: {p50:.2f}ms")
        print(f"  P95: {p95:.2f}ms")

        # With mocked dependencies, should be very fast
        assert p50 < 100, f"P50 search latency too high: {p50}ms"
        assert p95 < 200, f"P95 search latency too high: {p95}ms"


@pytest.mark.performance
class TestRAGPipelinePerformance:
    """Test end-to-end RAG pipeline performance"""

    @pytest.mark.asyncio
    async def test_rag_pipeline_latency(self):
        """Test full RAG pipeline latency"""
        from app.services.rag import rag_service

        mock_chunks = [
            {
                "chunk_id": f"chunk-{i}",
                "content": f"Test content {i}",
                "module": "module-01",
                "chapter": "chapter-01",
                "section": "section",
                "hybrid_score": 0.9 - i * 0.1,
                "chunk_type": "theoretical"
            }
            for i in range(5)
        ]

        mock_chat_response = Mock()
        mock_chat_response.choices = [Mock()]
        mock_chat_response.choices[0].message.content = "Test response"
        mock_chat_response.usage.total_tokens = 100

        latencies = []

        with patch.object(rag_service, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = mock_chunks

            with patch.object(
                rag_service.openai_client.chat.completions,
                'create',
                new_callable=AsyncMock,
                return_value=mock_chat_response
            ):
                # Measure 5 RAG queries
                for i in range(5):
                    start = time.time()
                    await rag_service.query(f"Test query {i}")
                    latency_ms = (time.time() - start) * 1000
                    latencies.append(latency_ms)

        p50 = median(latencies)
        p95 = sorted(latencies)[int(len(latencies) * 0.95)]
        avg = mean(latencies)

        print(f"\nEnd-to-End RAG Pipeline Latency:")
        print(f"  P50: {p50:.2f}ms")
        print(f"  P95: {p95:.2f}ms")
        print(f"  Avg: {avg:.2f}ms")

        # Target: < 200ms with mocking
        assert p50 < 200, f"P50 RAG latency too high: {p50}ms"
        assert p95 < 500, f"P95 RAG latency too high: {p95}ms"

    @pytest.mark.asyncio
    async def test_context_assembly_performance(self):
        """Test context assembly performance"""
        from app.services.rag import rag_service

        # Create 100 mock chunks
        chunks = [
            {
                "chunk_id": f"chunk-{i}",
                "content": f"This is test content for chunk {i}. " * 100,
                "module": f"module-{i % 4}",
                "chapter": f"chapter-{i % 10}",
                "section": f"section-{i}",
                "hybrid_score": 1.0 - (i * 0.01),
                "token_count": 400
            }
            for i in range(100)
        ]

        start = time.time()
        context = rag_service.assemble_context(chunks)
        elapsed_ms = (time.time() - start) * 1000

        print(f"\nContext Assembly Performance (100 chunks):")
        print(f"  Time: {elapsed_ms:.2f}ms")
        print(f"  Context length: {len(context)} chars")

        # Context assembly should be very fast (pure Python)
        assert elapsed_ms < 100, f"Context assembly too slow: {elapsed_ms}ms"
        assert len(context) > 0


@pytest.mark.performance
class TestConcurrency:
    """Test concurrent request handling"""

    @pytest.mark.asyncio
    async def test_concurrent_embeddings(self):
        """Test concurrent embedding generation"""
        from app.services.embeddings import embeddings_service

        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            # Generate 20 embeddings concurrently
            texts = [f"Concurrent text {i}" for i in range(20)]

            start = time.time()
            tasks = [embeddings_service.generate_embedding(text, use_cache=False) for text in texts]
            await asyncio.gather(*tasks)
            elapsed_ms = (time.time() - start) * 1000

            per_embedding_ms = elapsed_ms / 20

            print(f"\nConcurrent Embedding Generation (20 concurrent):")
            print(f"  Total: {elapsed_ms:.2f}ms")
            print(f"  Per embedding: {per_embedding_ms:.2f}ms")

            # Concurrent should be efficient
            assert elapsed_ms < 1000, f"Concurrent embeddings too slow: {elapsed_ms}ms"

    @pytest.mark.asyncio
    async def test_concurrent_rag_queries(self):
        """Test concurrent RAG queries"""
        from app.services.rag import rag_service

        mock_chunks = [{
            "chunk_id": "chunk-1",
            "content": "Test",
            "module": "m",
            "chapter": "c",
            "section": "s",
            "hybrid_score": 0.9,
            "chunk_type": "theoretical"
        }]

        mock_chat_response = Mock()
        mock_chat_response.choices = [Mock()]
        mock_chat_response.choices[0].message.content = "Response"
        mock_chat_response.usage.total_tokens = 100

        with patch.object(rag_service, 'retrieve_context', new_callable=AsyncMock) as mock_retrieve:
            mock_retrieve.return_value = mock_chunks

            with patch.object(
                rag_service.openai_client.chat.completions,
                'create',
                new_callable=AsyncMock,
                return_value=mock_chat_response
            ):
                # Run 10 concurrent RAG queries
                queries = [f"Query {i}" for i in range(10)]

                start = time.time()
                tasks = [rag_service.query(query) for query in queries]
                results = await asyncio.gather(*tasks)
                elapsed_ms = (time.time() - start) * 1000

                per_query_ms = elapsed_ms / 10

                print(f"\nConcurrent RAG Queries (10 concurrent):")
                print(f"  Total: {elapsed_ms:.2f}ms")
                print(f"  Per query: {per_query_ms:.2f}ms")

                # Should handle concurrent requests efficiently
                assert len(results) == 10
                assert per_query_ms < 500, f"Concurrent queries too slow: {per_query_ms}ms per query"


@pytest.mark.performance
class TestMemoryUsage:
    """Test memory usage (basic checks)"""

    @pytest.mark.asyncio
    async def test_cache_memory_limit(self):
        """Test that cache doesn't grow unbounded"""
        from app.services.embeddings import embeddings_service

        mock_response = Mock()
        mock_response.data = [Mock()]
        mock_response.data[0].embedding = [0.1] * 1536

        embeddings_service.clear_cache()

        with patch.object(
            embeddings_service.client.embeddings,
            'create',
            new_callable=AsyncMock,
            return_value=mock_response
        ):
            # Generate more embeddings than cache capacity
            for i in range(1500):
                await embeddings_service.generate_embedding(f"Text {i}")

            cache_size = len(embeddings_service._cache)

            print(f"\nCache Memory Management:")
            print(f"  Cache size after 1500 insertions: {cache_size}")
            print(f"  Max cache size: 1000")

            # Cache should be capped at 1000 entries
            assert cache_size <= 1000, f"Cache size exceeded limit: {cache_size}"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "performance"])
