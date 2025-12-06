"""Hybrid retrieval service combining vector and keyword search"""

from typing import List, Dict, Optional, Any
import logging
from sqlalchemy import text
from openai import AsyncOpenAI

from app.core.config import settings
from app.services.storage import storage_service

logger = logging.getLogger(__name__)


class RetrievalService:
    """
    Hybrid search combining vector similarity and BM25 keyword search

    Implements weighted scoring (70% vector, 30% BM25) with
    adaptive weighting based on query classification
    """

    def __init__(self):
        """Initialize retrieval service with storage connections"""
        self.vector_weight = settings.VECTOR_SEARCH_WEIGHT
        self.bm25_weight = settings.BM25_SEARCH_WEIGHT
        self.openai_client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)

    async def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding for query using OpenAI

        Args:
            query: Query text

        Returns:
            Query embedding vector (1536-dim)
        """
        try:
            response = await self.openai_client.embeddings.create(
                model=settings.OPENAI_EMBEDDING_MODEL,
                input=query
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Failed to generate query embedding: {e}")
            raise

    async def hybrid_search(
        self,
        query: str,
        top_k: int = 20,
        filters: Optional[Dict[str, Any]] = None,
        query_type: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Perform hybrid search combining vector and keyword matching

        Args:
            query: User query text
            top_k: Number of results to return (default 20)
            filters: Optional metadata filters (module, chunk_type, learning_level)
            query_type: Optional query type (if not provided, will classify)

        Returns:
            List of retrieval results with hybrid scores

        Process:
            1. Classify query and get adaptive weights
            2. Generate query embedding
            3. Vector search in Qdrant (top 50)
            4. BM25 search in Postgres (top 50)
            5. Normalize scores
            6. Combine with adaptive weights
            7. Rerank results
            8. Return top_k results
        """
        logger.info(f"Hybrid search for query: {query[:50]}...")

        # 1. Classify query and get adaptive weights
        if not query_type:
            query_type = await self.classify_query(query)

        weights = self.get_search_weights(query_type)
        vector_weight = weights["vector"]
        bm25_weight = weights["bm25"]

        logger.info(
            f"Query type: {query_type}, "
            f"weights: {vector_weight:.2f} vector, {bm25_weight:.2f} BM25"
        )

        # 2. Generate query embedding
        query_embedding = await self.generate_query_embedding(query)

        # 3. Vector search in Qdrant
        vector_results = await storage_service.search_vectors(
            query_embedding=query_embedding,
            limit=50,
            module=filters.get("module") if filters else None,
            chunk_type=filters.get("chunk_type") if filters else None,
            learning_level=filters.get("learning_level") if filters else None
        )

        logger.info(f"Vector search returned {len(vector_results)} results")

        # 4. BM25 search in Postgres
        bm25_results = await self.bm25_search(query, limit=50, filters=filters)

        logger.info(f"BM25 search returned {len(bm25_results)} results")

        # 5. Normalize and combine scores
        combined_scores = {}

        # Add vector scores (normalized to 0-1 range)
        max_vector_score = max([r["score"] for r in vector_results], default=1.0)
        for result in vector_results:
            chunk_id = result["payload"]["chunk_id"]
            normalized_score = result["score"] / max_vector_score if max_vector_score > 0 else 0
            combined_scores[chunk_id] = {
                "chunk_id": chunk_id,
                "vector_score": normalized_score,
                "bm25_score": 0.0,
                "payload": result["payload"]
            }

        # Add BM25 scores (already normalized by Postgres ts_rank)
        for result in bm25_results:
            chunk_id = result["chunk_id"]
            if chunk_id not in combined_scores:
                combined_scores[chunk_id] = {
                    "chunk_id": chunk_id,
                    "vector_score": 0.0,
                    "bm25_score": 0.0,
                    "content": result["content"],
                    "module": result["module"],
                    "chapter": result["chapter"],
                    "chunk_type": result.get("chunk_type", "")
                }
            combined_scores[chunk_id]["bm25_score"] = result["bm25_score"]
            combined_scores[chunk_id]["chunk_type"] = result.get("chunk_type", "")

        # 6. Calculate hybrid scores with adaptive weights
        for chunk_id in combined_scores:
            scores = combined_scores[chunk_id]
            hybrid_score = (
                vector_weight * scores["vector_score"] +
                bm25_weight * scores["bm25_score"]
            )
            combined_scores[chunk_id]["hybrid_score"] = hybrid_score

        # 7. Sort by hybrid score
        ranked_results = sorted(
            combined_scores.values(),
            key=lambda x: x["hybrid_score"],
            reverse=True
        )

        # 8. Enrich with full chunk data from database
        top_results = ranked_results[:min(top_k * 2, 50)]  # Get more for reranking
        enriched_results = await self.enrich_results(top_results)

        # 9. Rerank based on query type
        reranked_results = self.rerank_results(enriched_results, query_type)

        logger.info(f"Returning {len(reranked_results[:top_k])} hybrid search results")

        return reranked_results[:top_k]

    async def bm25_search(
        self,
        query: str,
        limit: int = 50,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        BM25 full-text search using Postgres

        Args:
            query: Query text
            limit: Maximum number of results
            filters: Optional metadata filters

        Returns:
            List of results with BM25 scores
        """
        try:
            # Build filter conditions
            filter_conditions = []
            params = {"query": query, "limit": limit}

            if filters:
                if filters.get("module"):
                    filter_conditions.append("module = :module")
                    params["module"] = filters["module"]
                if filters.get("chunk_type"):
                    filter_conditions.append("chunk_type = :chunk_type")
                    params["chunk_type"] = filters["chunk_type"]
                if filters.get("learning_level"):
                    filter_conditions.append("learning_level = :learning_level")
                    params["learning_level"] = filters["learning_level"]

            where_clause = ""
            if filter_conditions:
                where_clause = "AND " + " AND ".join(filter_conditions)

            # Execute BM25 search
            async with storage_service.get_session() as session:
                query_sql = text(f"""
                    SELECT
                        id::text as chunk_id,
                        content,
                        module,
                        chapter,
                        section,
                        chunk_type,
                        ts_rank(content_tsvector, plainto_tsquery('english', :query)) AS bm25_score
                    FROM chunks
                    WHERE content_tsvector @@ plainto_tsquery('english', :query)
                    {where_clause}
                    ORDER BY bm25_score DESC
                    LIMIT :limit
                """)

                result = await session.execute(query_sql, params)
                rows = result.fetchall()

                return [
                    {
                        "chunk_id": row[0],
                        "content": row[1],
                        "module": row[2],
                        "chapter": row[3],
                        "section": row[4],
                        "chunk_type": row[5],
                        "bm25_score": float(row[6])
                    }
                    for row in rows
                ]

        except Exception as e:
            logger.error(f"BM25 search failed: {e}")
            return []

    async def enrich_results(self, results: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Enrich search results with full chunk and document metadata

        Args:
            results: List of search results with chunk_ids

        Returns:
            Enriched results with full metadata
        """
        if not results:
            return []

        try:
            chunk_ids = [r["chunk_id"] for r in results]

            async with storage_service.get_session() as session:
                query_sql = text("""
                    SELECT
                        c.id::text as chunk_id,
                        c.content,
                        c.chunk_type,
                        c.module,
                        c.chapter,
                        c.section,
                        c.token_count,
                        d.title,
                        d.file_path,
                        d.learning_level,
                        d.prerequisites
                    FROM chunks c
                    JOIN documents d ON c.document_id = d.id
                    WHERE c.id::text = ANY(:chunk_ids)
                """)

                result = await session.execute(query_sql, {"chunk_ids": chunk_ids})
                rows = result.fetchall()

                # Create lookup map
                chunk_data = {
                    row[0]: {
                        "chunk_id": row[0],
                        "content": row[1],
                        "chunk_type": row[2],
                        "module": row[3],
                        "chapter": row[4],
                        "section": row[5],
                        "token_count": row[6],
                        "document_title": row[7],
                        "file_path": row[8],
                        "learning_level": row[9],
                        "prerequisites": row[10]
                    }
                    for row in rows
                }

                # Merge with original results
                enriched = []
                for r in results:
                    chunk_id = r["chunk_id"]
                    if chunk_id in chunk_data:
                        enriched_result = {
                            **chunk_data[chunk_id],
                            "vector_score": r.get("vector_score", 0.0),
                            "bm25_score": r.get("bm25_score", 0.0),
                            "hybrid_score": r.get("hybrid_score", 0.0)
                        }
                        enriched.append(enriched_result)

                return enriched

        except Exception as e:
            logger.error(f"Failed to enrich results: {e}")
            return results

    def rerank_results(
        self,
        results: List[Dict[str, Any]],
        query_type: str
    ) -> List[Dict[str, Any]]:
        """
        Rerank results based on query type preferences

        Args:
            results: List of retrieval results with scores
            query_type: Type of query

        Returns:
            Reranked list of results
        """
        if not results:
            return results

        reranked = results.copy()

        for result in reranked:
            boost = 0.0
            chunk_type = result.get("chunk_type", "")

            # Boost code chunks for code queries
            if query_type == "code" and chunk_type in ["code", "mixed"]:
                boost = 0.15

            # Boost theoretical chunks for conceptual queries
            elif query_type == "conceptual" and chunk_type == "theoretical":
                boost = 0.1

            # Boost mixed chunks for comparison queries
            elif query_type == "comparison" and chunk_type == "mixed":
                boost = 0.1

            # Boost code/mixed chunks for troubleshooting
            elif query_type == "troubleshooting" and chunk_type in ["code", "mixed"]:
                boost = 0.12

            # Apply boost to hybrid score
            result["hybrid_score"] = result.get("hybrid_score", 0.0) + boost
            result["rerank_boost"] = boost

        # Re-sort by boosted scores
        reranked.sort(key=lambda x: x.get("hybrid_score", 0.0), reverse=True)

        return reranked

    def get_search_weights(self, query_type: str) -> dict:
        """
        Get adaptive search weights based on query type

        Args:
            query_type: Type of query (conceptual, code, troubleshooting, comparison)

        Returns:
            Dictionary with vector_weight and bm25_weight
        """
        weights = {
            "conceptual": {"vector": 0.8, "bm25": 0.2},  # Semantic understanding
            "code": {"vector": 0.5, "bm25": 0.5},  # Exact matches important
            "troubleshooting": {"vector": 0.6, "bm25": 0.4},  # Error messages + context
            "comparison": {"vector": 0.7, "bm25": 0.3}  # Semantic similarity
        }

        return weights.get(query_type, {"vector": 0.7, "bm25": 0.3})

    async def classify_query(self, query: str) -> str:
        """
        Classify query type for adaptive weighting

        Args:
            query: User query text

        Returns:
            Query type: 'conceptual', 'code', 'troubleshooting', 'comparison'
        """
        query_lower = query.lower()

        # Check in order of specificity
        # Conceptual patterns
        conceptual_patterns = [
            "what is", "what are", "explain", "define", "how does",
            "why", "describe", "tell me about", "introduce"
        ]

        # Code patterns
        code_patterns = [
            "code for", "implement", "example of", "how to", "show me",
            "create", "build", "write", "develop", "sample code",
            "function", "class", "script"
        ]

        # Troubleshooting patterns
        troubleshooting_patterns = [
            "error", "fix", "not working", "issue with", "problem",
            "debug", "fails", "broken", "doesn't work", "trouble",
            "can't", "cannot", "won't", "crash"
        ]

        # Comparison patterns
        comparison_patterns = [
            "difference between", "vs", "versus", "compare",
            "which", "better", "advantages", "disadvantages",
            "pros and cons", "when to use"
        ]

        # Check each pattern category
        if any(pattern in query_lower for pattern in comparison_patterns):
            return "comparison"
        elif any(pattern in query_lower for pattern in troubleshooting_patterns):
            return "troubleshooting"
        elif any(pattern in query_lower for pattern in code_patterns):
            return "code"
        elif any(pattern in query_lower for pattern in conceptual_patterns):
            return "conceptual"
        else:
            return "conceptual"  # Default


# Global retrieval service instance
retrieval_service = RetrievalService()
