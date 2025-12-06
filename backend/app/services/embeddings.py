"""Embeddings generation service using OpenAI API"""

from typing import List
import hashlib
import logging
from functools import lru_cache
from openai import AsyncOpenAI, RateLimitError, APIError
import asyncio

from app.core.config import settings

logger = logging.getLogger(__name__)


class EmbeddingsService:
    """
    Service for generating text embeddings using OpenAI

    Uses text-embedding-3-small model (1536 dimensions)
    with LRU caching and retry logic for production reliability
    """

    def __init__(self):
        """Initialize embeddings service with OpenAI client"""
        self.client = AsyncOpenAI(
            api_key=settings.OPENAI_API_KEY,
            max_retries=settings.OPENAI_MAX_RETRIES,
            timeout=settings.OPENAI_TIMEOUT
        )
        self.model = settings.OPENAI_EMBEDDING_MODEL
        self.dimensions = settings.OPENAI_EMBEDDING_DIMENSIONS

        # In-memory cache for embeddings (hash -> embedding)
        self._cache = {}
        self._cache_hits = 0
        self._cache_misses = 0

        logger.info(f"EmbeddingsService initialized with model: {self.model}")

    def _get_cache_key(self, text: str) -> str:
        """
        Generate cache key from text

        Args:
            text: Text to hash

        Returns:
            MD5 hash of text (for cache lookup)
        """
        return hashlib.md5(text.encode('utf-8')).hexdigest()

    async def generate_embedding(
        self,
        text: str,
        use_cache: bool = True
    ) -> List[float]:
        """
        Generate embedding for single text with retry logic

        Args:
            text: Text to embed
            use_cache: Whether to use cache (default True)

        Returns:
            1536-dimensional embedding vector

        Raises:
            Exception: If OpenAI API call fails after retries
        """
        if not text or not text.strip():
            logger.warning("Empty text provided for embedding")
            return [0.0] * self.dimensions

        # Check cache first
        if use_cache:
            cache_key = self._get_cache_key(text)
            if cache_key in self._cache:
                self._cache_hits += 1
                logger.debug(f"Cache hit for text: {text[:50]}...")
                return self._cache[cache_key]
            self._cache_misses += 1

        # Generate embedding with retry logic
        retry_count = 0
        max_retries = settings.OPENAI_MAX_RETRIES

        while retry_count < max_retries:
            try:
                logger.debug(f"Generating embedding for text: {text[:50]}...")

                response = await self.client.embeddings.create(
                    model=self.model,
                    input=text,
                    dimensions=self.dimensions
                )

                embedding = response.data[0].embedding

                # Store in cache (LRU with max 1000 entries)
                if use_cache:
                    if len(self._cache) >= 1000:
                        # Remove oldest entry (simple FIFO for now)
                        oldest_key = next(iter(self._cache))
                        del self._cache[oldest_key]

                    self._cache[cache_key] = embedding

                logger.debug(f"Successfully generated embedding (dim: {len(embedding)})")
                return embedding

            except RateLimitError as e:
                retry_count += 1
                wait_time = 2 ** retry_count  # Exponential backoff
                logger.warning(
                    f"Rate limit hit, retrying in {wait_time}s "
                    f"(attempt {retry_count}/{max_retries}): {e}"
                )

                if retry_count < max_retries:
                    await asyncio.sleep(wait_time)
                else:
                    logger.error(f"Rate limit error after {max_retries} retries: {e}")
                    raise

            except APIError as e:
                retry_count += 1
                wait_time = 2 ** retry_count
                logger.warning(
                    f"API error, retrying in {wait_time}s "
                    f"(attempt {retry_count}/{max_retries}): {e}"
                )

                if retry_count < max_retries:
                    await asyncio.sleep(wait_time)
                else:
                    logger.error(f"API error after {max_retries} retries: {e}")
                    raise

            except Exception as e:
                logger.error(f"Unexpected error generating embedding: {e}")
                raise

        raise Exception(f"Failed to generate embedding after {max_retries} retries")

    async def batch_generate_embeddings(
        self,
        texts: List[str],
        use_cache: bool = True,
        batch_size: int = 100
    ) -> List[List[float]]:
        """
        Generate embeddings for batch of texts efficiently

        More efficient than individual calls for large batches.
        Handles OpenAI's limit of 2048 texts per batch.

        Args:
            texts: List of texts to embed
            use_cache: Whether to use cache (default True)
            batch_size: Number of texts per batch (default 100, max 2048)

        Returns:
            List of embedding vectors (same order as input)

        Raises:
            Exception: If OpenAI API call fails
        """
        if not texts:
            return []

        logger.info(f"Batch generating embeddings for {len(texts)} texts")

        # Check cache first and separate cached vs uncached
        embeddings_map = {}  # index -> embedding
        uncached_texts = {}  # index -> text

        for i, text in enumerate(texts):
            if not text or not text.strip():
                embeddings_map[i] = [0.0] * self.dimensions
                continue

            if use_cache:
                cache_key = self._get_cache_key(text)
                if cache_key in self._cache:
                    embeddings_map[i] = self._cache[cache_key]
                    self._cache_hits += 1
                else:
                    uncached_texts[i] = text
                    self._cache_misses += 1
            else:
                uncached_texts[i] = text

        logger.info(
            f"Cache stats: {len(embeddings_map)} hits, "
            f"{len(uncached_texts)} misses"
        )

        # Generate embeddings for uncached texts in batches
        if uncached_texts:
            uncached_indices = list(uncached_texts.keys())
            uncached_text_list = list(uncached_texts.values())

            # Process in batches (max 2048 per OpenAI batch, but use smaller batches)
            for batch_start in range(0, len(uncached_text_list), batch_size):
                batch_end = min(batch_start + batch_size, len(uncached_text_list))
                batch_texts = uncached_text_list[batch_start:batch_end]
                batch_indices = uncached_indices[batch_start:batch_end]

                logger.debug(
                    f"Processing batch {batch_start//batch_size + 1}: "
                    f"{len(batch_texts)} texts"
                )

                try:
                    # Call OpenAI API for batch
                    response = await self.client.embeddings.create(
                        model=self.model,
                        input=batch_texts,
                        dimensions=self.dimensions
                    )

                    # Map embeddings back to original indices
                    for j, embedding_data in enumerate(response.data):
                        original_index = batch_indices[j]
                        embedding = embedding_data.embedding
                        embeddings_map[original_index] = embedding

                        # Cache the embedding
                        if use_cache:
                            cache_key = self._get_cache_key(batch_texts[j])
                            if len(self._cache) >= 1000:
                                oldest_key = next(iter(self._cache))
                                del self._cache[oldest_key]
                            self._cache[cache_key] = embedding

                    logger.debug(f"Successfully generated {len(response.data)} embeddings")

                except Exception as e:
                    logger.error(f"Batch embedding generation failed: {e}")
                    # Fall back to individual generation for this batch
                    logger.warning(f"Falling back to individual generation for batch")
                    for j, text in enumerate(batch_texts):
                        original_index = batch_indices[j]
                        embedding = await self.generate_embedding(text, use_cache)
                        embeddings_map[original_index] = embedding

        # Return embeddings in original order
        result = [embeddings_map[i] for i in range(len(texts))]

        logger.info(f"Batch embedding complete: {len(result)} embeddings generated")

        return result

    def get_cache_stats(self) -> dict:
        """
        Get cache performance statistics

        Returns:
            Dictionary with cache stats (hits, misses, size, hit_rate)
        """
        total = self._cache_hits + self._cache_misses
        hit_rate = self._cache_hits / total if total > 0 else 0.0

        return {
            "cache_size": len(self._cache),
            "cache_hits": self._cache_hits,
            "cache_misses": self._cache_misses,
            "total_requests": total,
            "hit_rate": hit_rate
        }

    def clear_cache(self):
        """Clear the embedding cache"""
        self._cache.clear()
        self._cache_hits = 0
        self._cache_misses = 0
        logger.info("Embedding cache cleared")

    async def check_openai_health(self) -> bool:
        """
        Check OpenAI API health by attempting a simple embedding request

        Returns:
            True if OpenAI API is accessible, False otherwise
        """
        try:
            logger.debug("Checking OpenAI API health...")

            # Try to generate a simple embedding
            response = await self.client.embeddings.create(
                model=self.model,
                input="health check",
                dimensions=self.dimensions
            )

            if response and response.data and len(response.data) > 0:
                logger.debug("OpenAI API health check passed")
                return True
            else:
                logger.warning("OpenAI API health check failed: empty response")
                return False

        except Exception as e:
            logger.error(f"OpenAI API health check failed: {e}")
            return False


# Global embeddings service instance
embeddings_service = EmbeddingsService()
