"""SQLAlchemy database models for Postgres"""

from datetime import datetime
from sqlalchemy import (
    Column, String, Integer, DateTime, Text, ForeignKey, Boolean, Float,
    Index, UniqueConstraint, ARRAY
)
from sqlalchemy.dialects.postgresql import UUID, TSVECTOR, JSONB
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
import uuid

Base = declarative_base()


class Document(Base):
    """
    Book document (page/section) model

    Represents a single document from the Physical AI book
    (e.g., a chapter, section, or code example)
    """
    __tablename__ = "documents"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(Text, nullable=False)
    module = Column(String(100), nullable=False, index=True)
    chapter = Column(String(100))
    section = Column(String(100))
    content_type = Column(String(50))  # 'theoretical', 'code', 'tutorial', 'reference'
    file_path = Column(Text, unique=True)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow, nullable=False)
    version = Column(Integer, default=1, nullable=False)

    # Metadata fields
    author = Column(String(200))
    tags = Column(ARRAY(Text))
    prerequisites = Column(ARRAY(Text))
    learning_level = Column(String(20))  # 'beginner', 'intermediate', 'advanced'

    # Framework info (JSONB for flexible structure)
    frameworks = Column(JSONB)  # {"ros2": "humble", "gazebo": "sim-7"}

    # Full-text search vector
    content_tsvector = Column(TSVECTOR)

    # Relationships
    chunks = relationship("Chunk", back_populates="document", cascade="all, delete-orphan")

    __table_args__ = (
        Index("idx_documents_module", "module"),
        Index("idx_documents_chapter", "chapter"),
        Index("idx_documents_type", "content_type"),
        Index("idx_documents_level", "learning_level"),
        Index("idx_documents_fts", "content_tsvector", postgresql_using="gin"),
        Index("idx_documents_frameworks", "frameworks", postgresql_using="gin"),
        UniqueConstraint("file_path", name="unique_document_path"),
    )


class Chunk(Base):
    """
    Embedded content chunk model

    Represents a chunk of content with its embedding stored in Qdrant
    """
    __tablename__ = "chunks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    document_id = Column(UUID(as_uuid=True), ForeignKey("documents.id", ondelete="CASCADE"), nullable=False)

    # Content
    content = Column(Text, nullable=False)
    chunk_index = Column(Integer, nullable=False)
    token_count = Column(Integer)

    # Position in document
    start_char = Column(Integer)
    end_char = Column(Integer)

    # Vector reference (Qdrant point ID)
    vector_id = Column(Text, unique=True)

    # Metadata (denormalized for faster access)
    module = Column(String(100))
    chapter = Column(String(100))
    section = Column(String(100))
    chunk_type = Column(String(50))

    # Code-specific metadata
    language = Column(String(50))
    code_type = Column(String(50))  # 'function', 'class', 'config', 'command'

    # Search optimization
    content_tsvector = Column(TSVECTOR)

    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    # Relationships
    document = relationship("Document", back_populates="chunks")
    usage_records = relationship("ChunkUsage", back_populates="chunk", cascade="all, delete-orphan")

    __table_args__ = (
        Index("idx_chunks_document", "document_id"),
        Index("idx_chunks_vector_id", "vector_id"),
        Index("idx_chunks_module", "module"),
        Index("idx_chunks_type", "chunk_type"),
        Index("idx_chunks_fts", "content_tsvector", postgresql_using="gin"),
        UniqueConstraint("document_id", "chunk_index", name="unique_chunk_position"),
    )


class UserQuery(Base):
    """
    User query log model

    Tracks queries for analytics and improvement
    """
    __tablename__ = "user_queries"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(Text)
    query_text = Column(Text, nullable=False)
    query_type = Column(String(50))  # 'conceptual', 'code', 'troubleshooting', 'comparison'

    # Results
    retrieved_chunks = Column(ARRAY(UUID(as_uuid=True)))
    response_text = Column(Text)

    # Feedback
    helpful = Column(Boolean)
    feedback_text = Column(Text)

    # Performance metrics
    retrieval_time_ms = Column(Integer)
    generation_time_ms = Column(Integer)

    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    # Relationships
    chunk_usage = relationship("ChunkUsage", back_populates="query", cascade="all, delete-orphan")

    __table_args__ = (
        Index("idx_queries_user", "user_id"),
        Index("idx_queries_created", "created_at", postgresql_ops={"created_at": "DESC"}),
    )


class ChunkUsage(Base):
    """
    Chunk retrieval analytics model

    Tracks which chunks are retrieved for which queries
    """
    __tablename__ = "chunk_usage"

    chunk_id = Column(UUID(as_uuid=True), ForeignKey("chunks.id", ondelete="CASCADE"), nullable=False, primary_key=True)
    query_id = Column(UUID(as_uuid=True), ForeignKey("user_queries.id", ondelete="CASCADE"), nullable=False, primary_key=True)
    rank = Column(Integer)  # Position in retrieval results
    score = Column(Float)  # Relevance score
    used_in_context = Column(Boolean, default=False)

    # Relationships
    chunk = relationship("Chunk", back_populates="usage_records")
    query = relationship("UserQuery", back_populates="chunk_usage")

    __table_args__ = (
        Index("idx_usage_chunk", "chunk_id"),
        Index("idx_usage_score", "score", postgresql_ops={"score": "DESC"}),
    )
