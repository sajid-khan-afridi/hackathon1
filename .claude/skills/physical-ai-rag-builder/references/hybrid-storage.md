# Hybrid Storage Architecture: Neon Postgres + Qdrant

## Table of Contents
- [Architecture Overview](#architecture-overview)
- [Neon Postgres Schema](#neon-postgres-schema)
- [Qdrant Collections](#qdrant-collections)
- [Hybrid Query Patterns](#hybrid-query-patterns)
- [Synchronization Strategy](#synchronization-strategy)

## Architecture Overview

### Responsibility Split

**Neon Postgres**: Structured data, metadata, relationships
- Document metadata (title, module, chapter, type)
- User data (queries, feedback, preferences)
- Analytics (popular queries, chunk usage)
- Relationships (section hierarchies, prerequisites)

**Qdrant**: Vector embeddings, semantic search
- Chunk embeddings (OpenAI ada-002 or text-embedding-3)
- Multi-vector representations
- Fast approximate nearest neighbor search

### Why Hybrid?

1. **Postgres strengths**: ACID transactions, complex joins, filtering
2. **Qdrant strengths**: Vector similarity, fast retrieval, scalability
3. **Combined**: Best of both worlds for RAG

## Neon Postgres Schema

### Core Tables

```sql
-- Documents table (book structure)
CREATE TABLE documents (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    title TEXT NOT NULL,
    module VARCHAR(100) NOT NULL,
    chapter VARCHAR(100),
    section VARCHAR(100),
    content_type VARCHAR(50), -- theoretical | code | tutorial | reference
    file_path TEXT,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW(),
    version INTEGER DEFAULT 1,

    -- Metadata
    author VARCHAR(200),
    tags TEXT[],
    prerequisites TEXT[],
    learning_level VARCHAR(20), -- beginner | intermediate | advanced

    -- Framework info
    frameworks JSONB, -- {"ros2": "humble", "gazebo": "sim-7"}

    -- Full-text search
    content_tsvector TSVECTOR,

    CONSTRAINT unique_document_path UNIQUE (file_path)
);

-- Create indexes
CREATE INDEX idx_documents_module ON documents(module);
CREATE INDEX idx_documents_chapter ON documents(chapter);
CREATE INDEX idx_documents_type ON documents(content_type);
CREATE INDEX idx_documents_level ON documents(learning_level);
CREATE INDEX idx_documents_fts ON documents USING GIN(content_tsvector);
CREATE INDEX idx_documents_frameworks ON documents USING GIN(frameworks);

-- Chunks table (embedded content pieces)
CREATE TABLE chunks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    document_id UUID REFERENCES documents(id) ON DELETE CASCADE,

    -- Content
    content TEXT NOT NULL,
    chunk_index INTEGER NOT NULL,
    token_count INTEGER,

    -- Position in document
    start_char INTEGER,
    end_char INTEGER,

    -- Vector reference (Qdrant point ID)
    vector_id TEXT UNIQUE,

    -- Metadata (denormalized for faster access)
    module VARCHAR(100),
    chapter VARCHAR(100),
    section VARCHAR(100),
    chunk_type VARCHAR(50),

    -- Code-specific metadata
    language VARCHAR(50),
    code_type VARCHAR(50), -- function | class | config | command

    -- Search optimization
    content_tsvector TSVECTOR,

    created_at TIMESTAMP DEFAULT NOW(),

    CONSTRAINT unique_chunk_position UNIQUE (document_id, chunk_index)
);

-- Create indexes
CREATE INDEX idx_chunks_document ON chunks(document_id);
CREATE INDEX idx_chunks_vector_id ON chunks(vector_id);
CREATE INDEX idx_chunks_module ON chunks(module);
CREATE INDEX idx_chunks_type ON chunks(chunk_type);
CREATE INDEX idx_chunks_fts ON chunks USING GIN(content_tsvector);

-- User queries table (analytics & personalization)
CREATE TABLE user_queries (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id TEXT,
    query_text TEXT NOT NULL,
    query_type VARCHAR(50), -- conceptual | code | troubleshooting | comparison

    -- Results
    retrieved_chunks UUID[],
    response_text TEXT,

    -- Feedback
    helpful BOOLEAN,
    feedback_text TEXT,

    -- Performance metrics
    retrieval_time_ms INTEGER,
    generation_time_ms INTEGER,

    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_queries_user ON user_queries(user_id);
CREATE INDEX idx_queries_created ON user_queries(created_at DESC);

-- Chunk usage analytics
CREATE TABLE chunk_usage (
    chunk_id UUID REFERENCES chunks(id) ON DELETE CASCADE,
    query_id UUID REFERENCES user_queries(id) ON DELETE CASCADE,
    rank INTEGER, -- Position in retrieval results
    score FLOAT, -- Relevance score
    used_in_context BOOLEAN DEFAULT FALSE,

    PRIMARY KEY (chunk_id, query_id)
);

CREATE INDEX idx_usage_chunk ON chunk_usage(chunk_id);
CREATE INDEX idx_usage_score ON chunk_usage(score DESC);
```

### Triggers for Auto-updates

```sql
-- Auto-update tsvector on insert/update
CREATE FUNCTION update_content_tsvector() RETURNS TRIGGER AS $$
BEGIN
    NEW.content_tsvector := to_tsvector('english', NEW.content);
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER documents_tsvector_update
    BEFORE INSERT OR UPDATE ON documents
    FOR EACH ROW EXECUTE FUNCTION update_content_tsvector();

CREATE TRIGGER chunks_tsvector_update
    BEFORE INSERT OR UPDATE ON chunks
    FOR EACH ROW EXECUTE FUNCTION update_content_tsvector();

-- Auto-update updated_at timestamp
CREATE FUNCTION update_updated_at() RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at := NOW();
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER documents_updated_at
    BEFORE UPDATE ON documents
    FOR EACH ROW EXECUTE FUNCTION update_updated_at();
```

## Qdrant Collections

### Collection Configuration

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType

# Initialize client
client = QdrantClient(url="https://your-qdrant-instance.com", api_key="...")

# Create collection for chunk embeddings
client.create_collection(
    collection_name="physical_ai_chunks",
    vectors_config=VectorParams(
        size=1536,  # OpenAI ada-002 dimension
        distance=Distance.COSINE
    ),
    # Optimize for search speed
    hnsw_config={
        "m": 16,
        "ef_construct": 100,
    },
    # Enable payload indexing
    optimizers_config={
        "indexing_threshold": 10000,
    }
)
```

### Payload Schema

```python
# Each Qdrant point stores:
point_payload = {
    # Reference to Postgres
    "chunk_id": "uuid-string",
    "document_id": "uuid-string",

    # Metadata for filtering
    "module": "module-01-ros2",
    "chapter": "01-fundamentals",
    "section": "publisher-subscriber",
    "chunk_type": "code",
    "language": "python",
    "learning_level": "beginner",

    # Framework tags
    "frameworks": ["ros2-humble", "python3"],

    # For hybrid ranking
    "content_preview": "First 200 chars...",
    "token_count": 500,
}
```

### Payload Indexes

```python
# Create indexed fields for fast filtering
client.create_payload_index(
    collection_name="physical_ai_chunks",
    field_name="module",
    field_schema=PayloadSchemaType.KEYWORD
)

client.create_payload_index(
    collection_name="physical_ai_chunks",
    field_name="chunk_type",
    field_schema=PayloadSchemaType.KEYWORD
)

client.create_payload_index(
    collection_name="physical_ai_chunks",
    field_name="learning_level",
    field_schema=PayloadSchemaType.KEYWORD
)
```

## Hybrid Query Patterns

### Pattern 1: Vector Search + Postgres Filter

```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

async def search_with_filters(
    query_embedding: list[float],
    module: str = None,
    chunk_type: str = None,
    learning_level: str = None,
    limit: int = 20
):
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
    results = client.search(
        collection_name="physical_ai_chunks",
        query_vector=query_embedding,
        query_filter=Filter(must=must_conditions) if must_conditions else None,
        limit=limit,
        with_payload=True
    )

    return results
```

### Pattern 2: BM25 (Postgres) + Vector (Qdrant)

```python
async def hybrid_search(query: str, query_embedding: list[float]):
    # 1. BM25 search in Postgres
    bm25_results = await db.fetch("""
        SELECT
            chunk_id,
            ts_rank(content_tsvector, plainto_tsquery('english', $1)) AS bm25_score
        FROM chunks
        WHERE content_tsvector @@ plainto_tsquery('english', $1)
        ORDER BY bm25_score DESC
        LIMIT 50
    """, query)

    # 2. Vector search in Qdrant
    vector_results = client.search(
        collection_name="physical_ai_chunks",
        query_vector=query_embedding,
        limit=50
    )

    # 3. Combine and rerank
    combined_scores = {}

    # Add BM25 scores
    for row in bm25_results:
        combined_scores[row['chunk_id']] = {
            'bm25_score': float(row['bm25_score']),
            'vector_score': 0.0
        }

    # Add vector scores
    for result in vector_results:
        chunk_id = result.payload['chunk_id']
        if chunk_id not in combined_scores:
            combined_scores[chunk_id] = {'bm25_score': 0.0, 'vector_score': 0.0}
        combined_scores[chunk_id]['vector_score'] = result.score

    # Weighted combination
    WEIGHTS = {'bm25': 0.3, 'vector': 0.7}
    for chunk_id in combined_scores:
        scores = combined_scores[chunk_id]
        combined_scores[chunk_id]['final_score'] = (
            WEIGHTS['bm25'] * scores['bm25_score'] +
            WEIGHTS['vector'] * scores['vector_score']
        )

    # Sort by final score
    ranked_chunks = sorted(
        combined_scores.items(),
        key=lambda x: x[1]['final_score'],
        reverse=True
    )

    return ranked_chunks[:20]
```

### Pattern 3: Enriched Results

```python
async def get_enriched_results(chunk_ids: list[str]):
    """Fetch chunks with full metadata from Postgres"""
    return await db.fetch("""
        SELECT
            c.id,
            c.content,
            c.chunk_type,
            c.language,
            d.title,
            d.module,
            d.chapter,
            d.section,
            d.file_path,
            d.prerequisites
        FROM chunks c
        JOIN documents d ON c.document_id = d.id
        WHERE c.id = ANY($1)
    """, chunk_ids)
```

## Synchronization Strategy

### Ingestion Pipeline

```python
async def ingest_document(document_data: dict):
    """
    1. Insert document into Postgres
    2. Chunk the content
    3. Generate embeddings
    4. Store chunks in Postgres
    5. Store vectors in Qdrant
    6. Link via vector_id
    """
    async with db.transaction():
        # 1. Insert document
        doc_id = await db.fetchval("""
            INSERT INTO documents (title, module, chapter, content_type, ...)
            VALUES ($1, $2, $3, $4, ...)
            RETURNING id
        """, ...)

        # 2. Chunk content
        chunks = chunk_document(document_data['content'])

        # 3. Generate embeddings
        embeddings = await generate_embeddings([c['text'] for c in chunks])

        # 4-5. Insert chunks and vectors
        for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            # Create unique vector ID
            vector_id = f"{doc_id}_{idx}"

            # Insert into Postgres
            chunk_id = await db.fetchval("""
                INSERT INTO chunks (
                    document_id, content, chunk_index, vector_id, module, ...
                )
                VALUES ($1, $2, $3, $4, $5, ...)
                RETURNING id
            """, doc_id, chunk['text'], idx, vector_id, ...)

            # Insert into Qdrant
            client.upsert(
                collection_name="physical_ai_chunks",
                points=[{
                    "id": vector_id,
                    "vector": embedding,
                    "payload": {
                        "chunk_id": str(chunk_id),
                        "document_id": str(doc_id),
                        "module": document_data['module'],
                        # ... other metadata
                    }
                }]
            )
```

### Consistency Guarantees

1. **Postgres is source of truth** for metadata
2. **Qdrant mirrors Postgres** chunk IDs via payload
3. **Atomic operations**: Use Postgres transactions
4. **Reconciliation**: Periodic job to sync Qdrant with Postgres
5. **Deletion**: CASCADE delete in Postgres + cleanup in Qdrant

```python
async def delete_document(doc_id: str):
    """Delete document and all chunks from both systems"""
    async with db.transaction():
        # Get vector IDs before deletion
        vector_ids = await db.fetch("""
            SELECT vector_id FROM chunks WHERE document_id = $1
        """, doc_id)

        # Delete from Postgres (cascades to chunks)
        await db.execute("DELETE FROM documents WHERE id = $1", doc_id)

        # Delete from Qdrant
        client.delete(
            collection_name="physical_ai_chunks",
            points_selector=[v['vector_id'] for v in vector_ids]
        )
```
