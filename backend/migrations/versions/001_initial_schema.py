"""Initial schema for hybrid storage (Postgres + Qdrant)

Revision ID: 001
Revises:
Create Date: 2025-12-06

Creates:
- documents table with metadata and full-text search
- chunks table with vector references
- user_queries table for analytics
- chunk_usage table for tracking
- Triggers for auto-updating tsvector and timestamps
"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = '001'
down_revision = None
branch_labels = None
depends_on = None


def upgrade() -> None:
    """Create all tables, indexes, functions, and triggers"""

    # Enable UUID extension
    op.execute('CREATE EXTENSION IF NOT EXISTS "uuid-ossp";')

    # Create documents table
    op.create_table(
        'documents',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('title', sa.Text(), nullable=False),
        sa.Column('module', sa.String(length=100), nullable=False),
        sa.Column('chapter', sa.String(length=100)),
        sa.Column('section', sa.String(length=100)),
        sa.Column('content_type', sa.String(length=50)),
        sa.Column('file_path', sa.Text()),
        sa.Column('created_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('updated_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('version', sa.Integer(), nullable=False, server_default=sa.text('1')),
        sa.Column('author', sa.String(length=200)),
        sa.Column('tags', postgresql.ARRAY(sa.Text())),
        sa.Column('prerequisites', postgresql.ARRAY(sa.Text())),
        sa.Column('learning_level', sa.String(length=20)),
        sa.Column('frameworks', postgresql.JSONB()),
        sa.Column('content_tsvector', postgresql.TSVECTOR()),
        sa.UniqueConstraint('file_path', name='unique_document_path')
    )

    # Create indexes for documents
    op.create_index('idx_documents_module', 'documents', ['module'])
    op.create_index('idx_documents_chapter', 'documents', ['chapter'])
    op.create_index('idx_documents_type', 'documents', ['content_type'])
    op.create_index('idx_documents_level', 'documents', ['learning_level'])
    op.create_index('idx_documents_fts', 'documents', ['content_tsvector'], postgresql_using='gin')
    op.create_index('idx_documents_frameworks', 'documents', ['frameworks'], postgresql_using='gin')

    # Create chunks table
    op.create_table(
        'chunks',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('document_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('documents.id', ondelete='CASCADE'), nullable=False),
        sa.Column('content', sa.Text(), nullable=False),
        sa.Column('chunk_index', sa.Integer(), nullable=False),
        sa.Column('token_count', sa.Integer()),
        sa.Column('start_char', sa.Integer()),
        sa.Column('end_char', sa.Integer()),
        sa.Column('vector_id', sa.Text(), unique=True),
        sa.Column('module', sa.String(length=100)),
        sa.Column('chapter', sa.String(length=100)),
        sa.Column('section', sa.String(length=100)),
        sa.Column('chunk_type', sa.String(length=50)),
        sa.Column('language', sa.String(length=50)),
        sa.Column('code_type', sa.String(length=50)),
        sa.Column('content_tsvector', postgresql.TSVECTOR()),
        sa.Column('created_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()')),
        sa.UniqueConstraint('document_id', 'chunk_index', name='unique_chunk_position')
    )

    # Create indexes for chunks
    op.create_index('idx_chunks_document', 'chunks', ['document_id'])
    op.create_index('idx_chunks_vector_id', 'chunks', ['vector_id'])
    op.create_index('idx_chunks_module', 'chunks', ['module'])
    op.create_index('idx_chunks_type', 'chunks', ['chunk_type'])
    op.create_index('idx_chunks_fts', 'chunks', ['content_tsvector'], postgresql_using='gin')

    # Create user_queries table
    op.create_table(
        'user_queries',
        sa.Column('id', postgresql.UUID(as_uuid=True), primary_key=True, server_default=sa.text('gen_random_uuid()')),
        sa.Column('user_id', sa.Text()),
        sa.Column('query_text', sa.Text(), nullable=False),
        sa.Column('query_type', sa.String(length=50)),
        sa.Column('retrieved_chunks', postgresql.ARRAY(postgresql.UUID(as_uuid=True))),
        sa.Column('response_text', sa.Text()),
        sa.Column('helpful', sa.Boolean()),
        sa.Column('feedback_text', sa.Text()),
        sa.Column('retrieval_time_ms', sa.Integer()),
        sa.Column('generation_time_ms', sa.Integer()),
        sa.Column('created_at', sa.DateTime(), nullable=False, server_default=sa.text('NOW()'))
    )

    # Create indexes for user_queries
    op.create_index('idx_queries_user', 'user_queries', ['user_id'])
    op.create_index('idx_queries_created', 'user_queries', [sa.text('created_at DESC')])

    # Create chunk_usage table
    op.create_table(
        'chunk_usage',
        sa.Column('chunk_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('chunks.id', ondelete='CASCADE'), nullable=False, primary_key=True),
        sa.Column('query_id', postgresql.UUID(as_uuid=True), sa.ForeignKey('user_queries.id', ondelete='CASCADE'), nullable=False, primary_key=True),
        sa.Column('rank', sa.Integer()),
        sa.Column('score', sa.Float()),
        sa.Column('used_in_context', sa.Boolean(), server_default=sa.text('FALSE'))
    )

    # Create indexes for chunk_usage
    op.create_index('idx_usage_chunk', 'chunk_usage', ['chunk_id'])
    op.create_index('idx_usage_score', 'chunk_usage', [sa.text('score DESC')])

    # Create function to auto-update content_tsvector
    op.execute("""
        CREATE OR REPLACE FUNCTION update_content_tsvector() RETURNS TRIGGER AS $$
        BEGIN
            NEW.content_tsvector := to_tsvector('english', COALESCE(NEW.content, ''));
            RETURN NEW;
        END;
        $$ LANGUAGE plpgsql;
    """)

    # Create function to auto-update updated_at timestamp
    op.execute("""
        CREATE OR REPLACE FUNCTION update_updated_at() RETURNS TRIGGER AS $$
        BEGIN
            NEW.updated_at := NOW();
            RETURN NEW;
        END;
        $$ LANGUAGE plpgsql;
    """)

    # Create triggers for documents table
    op.execute("""
        CREATE TRIGGER documents_tsvector_update
            BEFORE INSERT OR UPDATE ON documents
            FOR EACH ROW EXECUTE FUNCTION update_content_tsvector();
    """)

    op.execute("""
        CREATE TRIGGER documents_updated_at
            BEFORE UPDATE ON documents
            FOR EACH ROW EXECUTE FUNCTION update_updated_at();
    """)

    # Create triggers for chunks table
    op.execute("""
        CREATE TRIGGER chunks_tsvector_update
            BEFORE INSERT OR UPDATE ON chunks
            FOR EACH ROW EXECUTE FUNCTION update_content_tsvector();
    """)


def downgrade() -> None:
    """Drop all tables, functions, and triggers"""

    # Drop triggers
    op.execute('DROP TRIGGER IF EXISTS chunks_tsvector_update ON chunks;')
    op.execute('DROP TRIGGER IF EXISTS documents_updated_at ON documents;')
    op.execute('DROP TRIGGER IF EXISTS documents_tsvector_update ON documents;')

    # Drop functions
    op.execute('DROP FUNCTION IF EXISTS update_updated_at();')
    op.execute('DROP FUNCTION IF EXISTS update_content_tsvector();')

    # Drop tables (in reverse order due to foreign keys)
    op.drop_table('chunk_usage')
    op.drop_table('user_queries')
    op.drop_table('chunks')
    op.drop_table('documents')
