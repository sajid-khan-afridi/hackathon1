"""Storage service for database and Qdrant operations"""

from typing import List, Optional, Dict, Any
import logging
from contextlib import asynccontextmanager

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy import select, text
from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance, VectorParams, PointStruct,
    PayloadSchemaType, Filter, FieldCondition, MatchValue
)

from app.core.config import settings
from app.models.database import Base, Document, Chunk, UserQuery, ChunkUsage

logger = logging.getLogger(__name__)


class StorageService:
    """
    Unified storage service for Postgres and Qdrant operations

    Manages connections, queries, and synchronization between
    relational metadata (Postgres) and vector embeddings (Qdrant)
    """

    def __init__(self):
        """Initialize storage service with database and Qdrant connections"""
        # Initialize async SQLAlchemy engine
        self.engine = create_async_engine(
            settings.ASYNC_DATABASE_URL,
            pool_size=settings.DB_POOL_SIZE,
            max_overflow=settings.DB_MAX_OVERFLOW,
            pool_timeout=settings.DB_POOL_TIMEOUT,
            pool_recycle=settings.DB_POOL_RECYCLE,
            echo=settings.DB_ECHO,
            pool_pre_ping=True,  # Verify connections before using
        )

        # Create async session factory
        self.async_session_maker = async_sessionmaker(
            self.engine,
            class_=AsyncSession,
            expire_on_commit=False,
        )

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30,
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME

        logger.info("StorageService initialized with async engine and Qdrant client")

    @asynccontextmanager
    async def get_session(self):
        """Get async database session with context manager"""
        async with self.async_session_maker() as session:
            try:
                yield session
            except Exception:
                await session.rollback()
                raise
            finally:
                await session.close()

    async def check_database_health(self) -> bool:
        """
        Check Postgres database connectivity

        Returns:
            True if database is accessible, False otherwise
        """
        try:
            async with self.get_session() as session:
                result = await session.execute(text("SELECT 1"))
                return result.scalar() == 1
        except Exception as e:
            logger.error(f"Database health check failed: {e}")
            return False

    async def check_qdrant_health(self) -> bool:
        """
        Check Qdrant connectivity

        Returns:
            True if Qdrant is accessible, False otherwise
        """
        try:
            # Try to get collection info
            collections = self.qdrant_client.get_collections()
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False

    async def create_qdrant_collection(self) -> bool:
        """
        Create Qdrant collection with proper configuration

        Returns:
            True if collection created or already exists, False on error
        """
        try:
            # Check if collection already exists
            collections = self.qdrant_client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name in collection_names:
                logger.info(f"Qdrant collection '{self.collection_name}' already exists")
                return True

            # Create collection
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=settings.QDRANT_VECTOR_SIZE,
                    distance=Distance.COSINE
                ),
                hnsw_config={
                    "m": settings.QDRANT_HNSW_M,
                    "ef_construct": settings.QDRANT_HNSW_EF_CONSTRUCT,
                },
                optimizers_config={
                    "indexing_threshold": 10000,
                }
            )

            # Create payload indexes for fast filtering
            self.qdrant_client.create_payload_index(
                collection_name=self.collection_name,
                field_name="module",
                field_schema=PayloadSchemaType.KEYWORD
            )

            self.qdrant_client.create_payload_index(
                collection_name=self.collection_name,
                field_name="chunk_type",
                field_schema=PayloadSchemaType.KEYWORD
            )

            self.qdrant_client.create_payload_index(
                collection_name=self.collection_name,
                field_name="learning_level",
                field_schema=PayloadSchemaType.KEYWORD
            )

            logger.info(f"Qdrant collection '{self.collection_name}' created successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to create Qdrant collection: {e}")
            return False

    async def get_documents(
        self,
        module: Optional[str] = None,
        chapter: Optional[str] = None,
        limit: int = 100
    ) -> List[Document]:
        """
        Get documents with optional filtering

        Args:
            module: Filter by module name
            chapter: Filter by chapter name
            limit: Maximum number of documents to return

        Returns:
            List of Document objects
        """
        try:
            async with self.get_session() as session:
                query = select(Document)

                if module:
                    query = query.where(Document.module == module)
                if chapter:
                    query = query.where(Document.chapter == chapter)

                query = query.limit(limit).order_by(Document.created_at.desc())

                result = await session.execute(query)
                return result.scalars().all()

        except Exception as e:
            logger.error(f"Failed to get documents: {e}")
            return []

    async def insert_chunk_with_vector(
        self,
        content: str,
        embedding: List[float],
        document_id: str,
        chunk_index: int,
        metadata: Dict[str, Any]
    ) -> Optional[str]:
        """
        Insert chunk into both Postgres and Qdrant (atomic dual-write)

        Args:
            content: Chunk text content
            embedding: Embedding vector (1536-dim)
            document_id: UUID of parent document
            chunk_index: Position in document
            metadata: Chunk metadata (module, chapter, section, etc.)

        Returns:
            Chunk ID (UUID) on success, None on failure

        Ensures consistency:
            - Generates unique chunk ID and vector ID
            - Inserts into Postgres (metadata + tsvector)
            - Inserts into Qdrant (embedding + payload)
            - Rolls back on failure
        """
        try:
            async with self.get_session() as session:
                # Create chunk in database
                chunk = Chunk(
                    document_id=document_id,
                    content=content,
                    chunk_index=chunk_index,
                    token_count=metadata.get('token_count'),
                    start_char=metadata.get('start_char'),
                    end_char=metadata.get('end_char'),
                    module=metadata.get('module'),
                    chapter=metadata.get('chapter'),
                    section=metadata.get('section'),
                    chunk_type=metadata.get('chunk_type'),
                    language=metadata.get('language'),
                    code_type=metadata.get('code_type'),
                )

                session.add(chunk)
                await session.flush()  # Get chunk ID without committing

                # Create vector ID (unique identifier for Qdrant)
                vector_id = f"{document_id}_{chunk_index}"
                chunk.vector_id = vector_id

                # Insert into Qdrant
                self.qdrant_client.upsert(
                    collection_name=self.collection_name,
                    points=[
                        PointStruct(
                            id=vector_id,
                            vector=embedding,
                            payload={
                                "chunk_id": str(chunk.id),
                                "document_id": str(document_id),
                                "module": metadata.get('module'),
                                "chapter": metadata.get('chapter'),
                                "section": metadata.get('section'),
                                "chunk_type": metadata.get('chunk_type'),
                                "language": metadata.get('language'),
                                "learning_level": metadata.get('learning_level'),
                                "frameworks": metadata.get('frameworks', []),
                                "content_preview": content[:200],
                                "token_count": metadata.get('token_count'),
                            }
                        )
                    ]
                )

                # Commit database transaction
                await session.commit()

                logger.info(f"Chunk {chunk.id} inserted successfully with vector {vector_id}")
                return str(chunk.id)

        except Exception as e:
            logger.error(f"Failed to insert chunk with vector: {e}")
            # Cleanup: Try to remove from Qdrant if database insert failed
            try:
                self.qdrant_client.delete(
                    collection_name=self.collection_name,
                    points_selector=[vector_id]
                )
            except:
                pass
            return None

    async def search_vectors(
        self,
        query_embedding: List[float],
        limit: int = 20,
        module: Optional[str] = None,
        chunk_type: Optional[str] = None,
        learning_level: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search Qdrant for similar vectors with optional filtering

        Args:
            query_embedding: Query vector (1536-dim)
            limit: Number of results to return
            module: Filter by module
            chunk_type: Filter by chunk type
            learning_level: Filter by learning level

        Returns:
            List of search results with scores and payloads
        """
        try:
            # Build filter conditions
            must_conditions = []
            if module:
                must_conditions.append(
                    FieldCondition(key="module", match=MatchValue(value=module))
                )
            if chunk_type:
                must_conditions.append(
                    FieldCondition(key="chunk_type", match=MatchValue(value=chunk_type))
                )
            if learning_level:
                must_conditions.append(
                    FieldCondition(key="learning_level", match=MatchValue(value=learning_level))
                )

            # Search Qdrant
            results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=Filter(must=must_conditions) if must_conditions else None,
                limit=limit,
                with_payload=True
            )

            return [
                {
                    "vector_id": result.id,
                    "score": result.score,
                    "payload": result.payload
                }
                for result in results
            ]

        except Exception as e:
            logger.error(f"Vector search failed: {e}")
            return []

    async def close(self):
        """Close database connections"""
        await self.engine.dispose()
        logger.info("Database engine disposed")


# Global storage service instance
storage_service = StorageService()
