"""RAG pipeline orchestration service"""

from typing import List, Dict, Optional, Any
import logging
import time
import hashlib
import re
from datetime import datetime
from uuid import uuid4
from openai import AsyncOpenAI

from app.core.config import settings
from app.models.schemas import ChatResponse, Citation
from app.services.embeddings import embeddings_service
from app.services.retrieval import retrieval_service
from app.services.storage import storage_service
from app.models.database import UserQuery, ChunkUsage
from sqlalchemy import text

logger = logging.getLogger(__name__)


class RAGService:
    """
    Complete RAG pipeline orchestration

    Coordinates query processing, retrieval, context assembly,
    generation, and citation extraction
    """

    def __init__(self):
        """Initialize RAG service with dependencies"""
        self.openai_client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)
        self.chat_model = settings.OPENAI_CHAT_MODEL
        self.max_context_tokens = settings.MAX_CONTEXT_TOKENS
        self.max_response_tokens = settings.MAX_RESPONSE_TOKENS
        logger.info(f"RAGService initialized with model: {self.chat_model}")

    async def query(
        self,
        query_text: str,
        user_id: Optional[str] = None,
        top_k: int = 8
    ) -> Dict[str, Any]:
        """
        End-to-end RAG pipeline for query answering

        Args:
            query_text: User's natural language question
            user_id: Optional user identifier
            top_k: Number of chunks to retrieve (default 8)

        Returns:
            Dictionary with response, sources, and performance metrics

        Pipeline:
            1. Process query (validate, classify)
            2. Retrieve context (hybrid search)
            3. Assemble context (deduplicate, merge, truncate)
            4. Build prompt (system message + context + query)
            5. Generate response (OpenAI chat completion)
            6. Extract citations (map to sources)
            7. Log to database (analytics)
        """
        logger.info(f"RAG query: {query_text[:100]}...")

        start_time = time.time()

        # 1. Process query
        processed_query = await self.process_query(query_text)
        query_type = processed_query["query_type"]

        # 2. Retrieve context
        retrieval_start = time.time()
        chunks = await self.retrieve_context(
            query_text,
            top_k=top_k,
            query_type=query_type
        )
        retrieval_time_ms = int((time.time() - retrieval_start) * 1000)

        logger.info(f"Retrieved {len(chunks)} chunks in {retrieval_time_ms}ms")

        # 3. Assemble context
        context = self.assemble_context(chunks)

        # 4. Generate response
        generation_start = time.time()
        response_text, raw_response = await self.generate_response(
            query_text,
            context,
            query_type
        )
        generation_time_ms = int((time.time() - generation_start) * 1000)

        logger.info(f"Generated response in {generation_time_ms}ms")

        # 5. Extract citations
        citations = self.extract_citations(response_text, chunks)

        # 6. Calculate total time
        total_time_ms = int((time.time() - start_time) * 1000)

        # 7. Log to database (async, non-blocking)
        try:
            await self.log_query(
                query_text=query_text,
                query_type=query_type,
                user_id=user_id,
                chunks=chunks,
                response_text=response_text,
                retrieval_time_ms=retrieval_time_ms,
                generation_time_ms=generation_time_ms
            )
        except Exception as e:
            logger.error(f"Failed to log query: {e}")

        # 8. Return response
        return {
            "response": response_text,
            "sources": [self._citation_to_dict(c) for c in citations],
            "query_type": query_type,
            "retrieval_time_ms": retrieval_time_ms,
            "generation_time_ms": generation_time_ms,
            "total_time_ms": total_time_ms,
            "chunks_used": len(chunks)
        }

    async def process_query(self, query: str) -> Dict[str, Any]:
        """
        Process and validate query

        Args:
            query: User query text

        Returns:
            Processed query object with metadata
        """
        # Validate query
        if not query or not query.strip():
            raise ValueError("Query cannot be empty")

        query = query.strip()

        if len(query) > settings.MAX_QUERY_LENGTH:
            raise ValueError(f"Query too long (max {settings.MAX_QUERY_LENGTH} chars)")

        # Classify query type
        query_type = await retrieval_service.classify_query(query)

        return {
            "query": query,
            "query_type": query_type
        }

    async def retrieve_context(
        self,
        query: str,
        top_k: int = 8,
        query_type: Optional[str] = None,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context chunks using hybrid search

        Args:
            query: User query
            top_k: Number of chunks to retrieve
            query_type: Optional query type (if already classified)
            filters: Optional metadata filters

        Returns:
            List of enriched chunks with scores
        """
        return await retrieval_service.hybrid_search(
            query=query,
            top_k=top_k,
            filters=filters,
            query_type=query_type
        )

    def assemble_context(self, chunks: List[Dict[str, Any]]) -> str:
        """
        Assemble context from retrieved chunks

        Process:
            1. Deduplicate chunks (by content hash)
            2. Merge overlapping chunks from same section
            3. Order by relevance score
            4. Truncate to max tokens
            5. Format with section headers

        Args:
            chunks: List of retrieved chunks

        Returns:
            Formatted context string
        """
        if not chunks:
            return "No relevant context found."

        # 1. Deduplicate by content hash
        seen_hashes = set()
        unique_chunks = []

        for chunk in chunks:
            content = chunk.get("content", "")
            content_hash = hashlib.md5(content.encode()).hexdigest()

            if content_hash not in seen_hashes:
                seen_hashes.add(content_hash)
                unique_chunks.append(chunk)

        logger.info(f"Deduplicated {len(chunks)} -> {len(unique_chunks)} chunks")

        # 2. Sort by hybrid score (already sorted, but ensure)
        unique_chunks.sort(key=lambda x: x.get("hybrid_score", 0), reverse=True)

        # 3. Format context with section headers
        context_parts = []
        total_tokens = 0
        max_tokens = self.max_context_tokens

        for i, chunk in enumerate(unique_chunks):
            # Estimate tokens (rough: 1 token ≈ 4 chars)
            chunk_tokens = len(chunk.get("content", "")) // 4

            if total_tokens + chunk_tokens > max_tokens:
                logger.warning(
                    f"Context truncated at {i} chunks "
                    f"({total_tokens} tokens, max: {max_tokens})"
                )
                break

            # Format chunk with metadata header
            module = chunk.get("module", "Unknown")
            chapter = chunk.get("chapter", "Unknown")
            section = chunk.get("section", "")
            score = chunk.get("hybrid_score", 0)

            header = f"\n--- Source {i+1}: {module} > {chapter}"
            if section:
                header += f" > {section}"
            header += f" (relevance: {score:.2f}) ---\n"

            context_parts.append(header)
            context_parts.append(chunk.get("content", ""))
            context_parts.append("\n")

            total_tokens += chunk_tokens

        context = "".join(context_parts)

        logger.info(
            f"Assembled context: {len(unique_chunks)} chunks, "
            f"~{total_tokens} tokens"
        )

        return context

    def build_prompt(
        self,
        query: str,
        context: str,
        query_type: str
    ) -> List[Dict[str, str]]:
        """
        Build prompt messages for OpenAI chat completion

        Args:
            query: User query
            context: Assembled context
            query_type: Type of query

        Returns:
            List of message dictionaries (system, user)
        """
        # System message tailored to query type
        system_messages = {
            "conceptual": (
                "You are an expert AI tutor for Physical AI and robotics. "
                "Your role is to explain concepts clearly and comprehensively "
                "using the provided book content. "
                "Use concrete examples from the book to illustrate key points. "
                "Always cite your sources using the format: [Module X > Chapter Y]."
            ),
            "code": (
                "You are an expert programming instructor for Physical AI and robotics. "
                "Provide working code examples with clear explanations of what each part does. "
                "Use code from the book when available, and explain best practices. "
                "Always cite your sources using the format: [Module X > Chapter Y]."
            ),
            "troubleshooting": (
                "You are an expert debugging assistant for Physical AI and robotics. "
                "Diagnose the issue based on the error description and suggest specific fixes. "
                "Reference known issues and solutions from the book when applicable. "
                "Always cite your sources using the format: [Module X > Chapter Y]."
            ),
            "comparison": (
                "You are an expert technical advisor for Physical AI and robotics. "
                "Compare the concepts/tools objectively, listing pros and cons. "
                "Help the user make informed decisions based on their use case. "
                "Always cite your sources using the format: [Module X > Chapter Y]."
            )
        }

        system_message = system_messages.get(
            query_type,
            system_messages["conceptual"]
        )

        # Add context injection instructions
        system_message += (
            "\n\nIMPORTANT: Base your answer ONLY on the provided context below. "
            "If the context doesn't contain enough information to fully answer "
            "the question, acknowledge what you can answer and what's missing. "
            "Never make up information not present in the context."
        )

        # User message with context and query
        user_message = f"""Context from Physical AI book:

{context}

---

Question: {query}

Please provide a comprehensive answer based on the context above. Remember to cite your sources."""

        messages = [
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ]

        return messages

    async def generate_response(
        self,
        query: str,
        context: str,
        query_type: str
    ) -> tuple[str, Any]:
        """
        Generate response using OpenAI chat completion

        Args:
            query: User query
            context: Assembled context
            query_type: Type of query

        Returns:
            Tuple of (response_text, raw_response_object)
        """
        messages = self.build_prompt(query, context, query_type)

        try:
            response = await self.openai_client.chat.completions.create(
                model=self.chat_model,
                messages=messages,
                temperature=0.3,  # Factual responses
                max_tokens=self.max_response_tokens,
                top_p=0.9,
                frequency_penalty=0.0,
                presence_penalty=0.0
            )

            response_text = response.choices[0].message.content

            logger.info(
                f"Generated response: {len(response_text)} chars, "
                f"tokens used: {response.usage.total_tokens}"
            )

            return response_text, response

        except Exception as e:
            logger.error(f"Response generation failed: {e}")
            raise

    def extract_citations(
        self,
        response_text: str,
        chunks: List[Dict[str, Any]]
    ) -> List[Citation]:
        """
        Extract citations from response and map to source chunks

        Args:
            response_text: Generated response text
            chunks: Retrieved chunks used for generation

        Returns:
            List of Citation objects
        """
        citations = []
        seen_citations = set()

        # Pattern to match citations: [Module X > Chapter Y] or [Module X > Chapter Y > Section Z]
        citation_pattern = r'\[([^\]]+)\]'

        matches = re.findall(citation_pattern, response_text)

        for match in matches:
            # Parse citation text
            parts = [p.strip() for p in match.split('>')]

            if len(parts) < 2:
                continue  # Not a valid citation format

            module = parts[0] if len(parts) > 0 else ""
            chapter = parts[1] if len(parts) > 1 else ""
            section = parts[2] if len(parts) > 2 else ""

            # Find matching chunk
            for chunk in chunks:
                chunk_module = chunk.get("module", "")
                chunk_chapter = chunk.get("chapter", "")
                chunk_section = chunk.get("section", "")

                # Fuzzy match (case-insensitive, partial match)
                module_match = module.lower() in chunk_module.lower() if module else True
                chapter_match = chapter.lower() in chunk_chapter.lower() if chapter else True
                section_match = section.lower() in chunk_section.lower() if section and chunk_section else True

                if module_match and chapter_match and section_match:
                    # Create citation key to avoid duplicates
                    citation_key = f"{chunk_module}|{chunk_chapter}|{chunk_section}"

                    if citation_key not in seen_citations:
                        seen_citations.add(citation_key)

                        # Build URL fragment
                        url_fragment = self._build_url_fragment(
                            chunk_module,
                            chunk_chapter,
                            chunk_section
                        )

                        citation = Citation(
                            module=chunk_module,
                            chapter=chunk_chapter,
                            section=chunk_section or "Introduction",
                            url_fragment=url_fragment
                        )

                        citations.append(citation)
                        break

        # If no citations extracted, create from top chunks
        if not citations:
            logger.warning("No citations found in response, using top chunks")
            for chunk in chunks[:3]:  # Top 3 most relevant
                citation_key = (
                    f"{chunk.get('module', '')}|"
                    f"{chunk.get('chapter', '')}|"
                    f"{chunk.get('section', '')}"
                )

                if citation_key not in seen_citations:
                    seen_citations.add(citation_key)

                    citation = Citation(
                        module=chunk.get("module", "Unknown"),
                        chapter=chunk.get("chapter", "Unknown"),
                        section=chunk.get("section", "Introduction"),
                        url_fragment=self._build_url_fragment(
                            chunk.get("module", ""),
                            chunk.get("chapter", ""),
                            chunk.get("section", "")
                        )
                    )

                    citations.append(citation)

        logger.info(f"Extracted {len(citations)} citations")

        return citations

    def _build_url_fragment(
        self,
        module: str,
        chapter: str,
        section: str
    ) -> str:
        """
        Build URL fragment for citation linking

        Args:
            module: Module name
            chapter: Chapter name
            section: Section name

        Returns:
            URL fragment (e.g., "#ros2-fundamentals-publisher")
        """
        # Convert to URL-friendly format
        parts = []

        if module:
            # Extract module number/name (e.g., "module-01-ros2" -> "ros2")
            module_clean = module.split('-')[-1] if '-' in module else module
            parts.append(module_clean)

        if chapter:
            # Extract chapter name (e.g., "01-fundamentals" -> "fundamentals")
            chapter_clean = chapter.split('-')[-1] if '-' in chapter else chapter
            parts.append(chapter_clean)

        if section:
            section_clean = section.lower().replace(' ', '-').replace('_', '-')
            parts.append(section_clean)

        fragment = '-'.join(parts).lower()

        # Remove special characters
        fragment = re.sub(r'[^a-z0-9\-]', '', fragment)

        return f"#{fragment}" if fragment else "#"

    def _citation_to_dict(self, citation: Citation) -> Dict[str, str]:
        """Convert Citation object to dictionary"""
        return {
            "module": citation.module,
            "chapter": citation.chapter,
            "section": citation.section,
            "url_fragment": citation.url_fragment
        }

    async def log_query(
        self,
        query_text: str,
        query_type: str,
        user_id: Optional[str],
        chunks: List[Dict[str, Any]],
        response_text: str,
        retrieval_time_ms: int,
        generation_time_ms: int
    ):
        """
        Log query to database for analytics

        Args:
            query_text: User query
            query_type: Classified query type
            user_id: Optional user ID
            chunks: Retrieved chunks
            response_text: Generated response
            retrieval_time_ms: Retrieval time in milliseconds
            generation_time_ms: Generation time in milliseconds
        """
        try:
            async with storage_service.get_session() as session:
                # Create user query record
                query_id = uuid4()
                chunk_ids = [chunk.get("chunk_id") for chunk in chunks if chunk.get("chunk_id")]

                query_record = UserQuery(
                    id=query_id,
                    user_id=user_id,
                    query_text=query_text,
                    query_type=query_type,
                    retrieved_chunks=chunk_ids[:8],  # Store top 8
                    response_text=response_text,
                    retrieval_time_ms=retrieval_time_ms,
                    generation_time_ms=generation_time_ms,
                    created_at=datetime.utcnow()
                )

                session.add(query_record)

                # Create chunk usage records
                for rank, chunk in enumerate(chunks[:8]):
                    chunk_id = chunk.get("chunk_id")
                    if chunk_id:
                        usage_record = ChunkUsage(
                            chunk_id=chunk_id,
                            query_id=query_id,
                            rank=rank,
                            score=chunk.get("hybrid_score", 0.0),
                            used_in_context=True
                        )
                        session.add(usage_record)

                await session.commit()

                logger.info(f"Logged query {query_id} to database")

        except Exception as e:
            logger.error(f"Failed to log query to database: {e}")
            # Don't raise - logging failure shouldn't break the pipeline


# Global RAG service instance
rag_service = RAGService()


# Test utilities
async def test_rag_pipeline():
    """
    Test end-to-end RAG pipeline with sample queries

    Tests all 4 query types and validates:
    - Query classification accuracy
    - Response generation
    - Citation extraction
    - Performance targets (<3s)
    """
    test_queries = [
        ("What is ROS2?", "conceptual"),
        ("Show me a ROS2 publisher example", "code"),
        ("How to fix 'node not found' error?", "troubleshooting"),
        ("Difference between Gazebo and Unity?", "comparison"),
    ]

    print("\n" + "="*80)
    print("RAG PIPELINE TEST")
    print("="*80 + "\n")

    for query, expected_type in test_queries:
        print(f"\nQuery: {query}")
        print(f"Expected type: {expected_type}")
        print("-" * 80)

        try:
            result = await rag_service.query(query)

            # Validate query type
            actual_type = result['query_type']
            type_match = "✓" if actual_type == expected_type else "✗"
            print(f"{type_match} Type: {actual_type} (expected: {expected_type})")

            # Validate response
            response_length = len(result['response'])
            print(f"✓ Response: {response_length} chars")
            print(f"  Preview: {result['response'][:200]}...")

            # Validate citations
            num_sources = len(result['sources'])
            sources_ok = "✓" if num_sources > 0 else "✗"
            print(f"{sources_ok} Sources: {num_sources} citations")

            if result['sources']:
                for i, source in enumerate(result['sources'][:3]):
                    print(f"  [{i+1}] {source['module']} > {source['chapter']} > {source['section']}")

            # Validate performance
            total_time = result['total_time_ms']
            time_ok = "✓" if total_time < 3000 else "✗"
            print(f"{time_ok} Time: {total_time}ms (target: <3000ms)")
            print(f"  - Retrieval: {result['retrieval_time_ms']}ms")
            print(f"  - Generation: {result['generation_time_ms']}ms")

            # Validate chunks
            print(f"✓ Chunks used: {result['chunks_used']}")

            # Overall success
            success = (
                actual_type == expected_type and
                num_sources > 0 and
                total_time < 3000
            )

            status = "✓ PASSED" if success else "✗ FAILED"
            print(f"\n{status}")

        except Exception as e:
            print(f"✗ FAILED: {e}")
            logger.exception(f"Test failed for query: {query}")

        print("\n" + "="*80)

    print("\nTest complete!")


async def test_embeddings():
    """Test embeddings service"""
    print("\n" + "="*80)
    print("EMBEDDINGS SERVICE TEST")
    print("="*80 + "\n")

    test_text = "What is ROS2 and how does it work?"

    print(f"Generating embedding for: {test_text}")

    try:
        embedding = await embeddings_service.generate_embedding(test_text)
        print(f"✓ Embedding generated: {len(embedding)} dimensions")
        print(f"✓ Sample values: {embedding[:5]}")

        # Test cache
        embedding2 = await embeddings_service.generate_embedding(test_text)
        stats = embeddings_service.get_cache_stats()
        print(f"✓ Cache working: {stats['cache_hits']} hits, {stats['cache_misses']} misses")
        print(f"✓ Hit rate: {stats['hit_rate']:.2%}")

        print("\n✓ PASSED")

    except Exception as e:
        print(f"✗ FAILED: {e}")
        logger.exception("Embeddings test failed")

    print("\n" + "="*80)


async def test_retrieval():
    """Test retrieval service"""
    print("\n" + "="*80)
    print("RETRIEVAL SERVICE TEST")
    print("="*80 + "\n")

    test_query = "What is ROS2?"

    print(f"Testing retrieval for: {test_query}\n")

    try:
        # Test query classification
        query_type = await retrieval_service.classify_query(test_query)
        print(f"✓ Query classified as: {query_type}")

        # Test weights
        weights = retrieval_service.get_search_weights(query_type)
        print(f"✓ Search weights: {weights}")

        # Test hybrid search
        results = await retrieval_service.hybrid_search(
            test_query,
            top_k=5,
            query_type=query_type
        )

        print(f"✓ Retrieved {len(results)} chunks")

        for i, result in enumerate(results[:3]):
            print(f"\n  Chunk {i+1}:")
            print(f"    Module: {result.get('module', 'N/A')}")
            print(f"    Chapter: {result.get('chapter', 'N/A')}")
            print(f"    Score: {result.get('hybrid_score', 0):.4f}")
            print(f"    Content: {result.get('content', '')[:100]}...")

        print("\n✓ PASSED")

    except Exception as e:
        print(f"✗ FAILED: {e}")
        logger.exception("Retrieval test failed")

    print("\n" + "="*80)
