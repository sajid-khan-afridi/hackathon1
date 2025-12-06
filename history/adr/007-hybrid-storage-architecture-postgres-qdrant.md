# ADR-007: Hybrid Storage Architecture with Postgres and Qdrant

> **Scope**: Dual-database architecture using Neon Postgres for structured metadata and Qdrant for vector embeddings in the RAG system.

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-rag-chatbot
- **Context:** Physical AI book RAG chatbot system

<!-- Significance checklist:
     1) Impact: Yes - Foundational infrastructure decision affecting performance, cost, and scalability
     2) Alternatives: Yes - Single database (Postgres with pgvector) vs dedicated vector DB vs hybrid
     3) Scope: Yes - Cross-cutting concern affecting storage, retrieval, synchronization, and data integrity
-->

## Decision

We will use a **hybrid storage architecture** combining:

**Neon Postgres (Serverless Postgres with pgvector)**:
- **Role**: Source of truth for all structured metadata and relational data
- **Stores**: Documents, chunks, user queries, analytics, relationships
- **Capabilities**: ACID transactions, complex joins, full-text search (BM25 via tsvector)
- **Provider**: Neon Serverless Postgres (free tier: 512MB storage, autoscaling compute)

**Qdrant (Open-source vector database)**:
- **Role**: Fast vector similarity search with metadata filtering
- **Stores**: Chunk embeddings (1536-dim vectors) with payload linking to Postgres
- **Capabilities**: Approximate nearest neighbor (ANN) search, HNSW indexing, payload filtering
- **Deployment**: Docker (local dev), Qdrant Cloud free tier (production: 1GB storage, 100K vectors)

**Synchronization Strategy**:
- **Postgres as Source of Truth**: All metadata managed in Postgres with foreign key constraints
- **Qdrant Mirrors Vectors**: Each Qdrant point has payload with `chunk_id` and `document_id` referencing Postgres
- **Atomic Operations**: Postgres transaction + Qdrant upsert in single operation (rollback on failure)
- **Reconciliation**: Periodic job to detect and fix inconsistencies between databases
- **Unique Linking**: `vector_id` field in Postgres chunks table links to Qdrant point ID

## Consequences

### Positive

- **Best-of-Both-Worlds**: Leverage Postgres strengths (ACID, joins, BM25) and Qdrant strengths (fast vector search)
- **Hybrid Search Performance**: Combine vector similarity (Qdrant) with keyword matching (Postgres BM25) for superior retrieval accuracy
- **Rich Metadata Queries**: Postgres enables complex filtering (module, chapter, prerequisites, learning level) before vector search
- **Analytics Capability**: User queries, chunk usage, and retrieval patterns stored in Postgres for future optimization
- **Cost Efficiency**: Free tiers for both (Neon 512MB + Qdrant 1GB sufficient for ~10K chunks)
- **Operational Simplicity**: Managed services (Neon Serverless, Qdrant Cloud) minimize DevOps overhead
- **Open Source**: Qdrant is open-source (self-hostable, no vendor lock-in)
- **Performance**: Qdrant's HNSW indexing delivers sub-50ms vector search at 10K chunk scale

### Negative

- **Two Databases to Manage**: Increased complexity vs single-database solution
  - **Mitigation**: Postgres as source of truth simplifies ownership; reconciliation job ensures consistency
- **Synchronization Risk**: Potential for Postgres-Qdrant data inconsistencies
  - **Mitigation**: Atomic operations; reconciliation job; Postgres CASCADE deletes trigger Qdrant cleanup
- **Network Latency**: Two separate database calls per query (Postgres metadata + Qdrant vector search)
  - **Mitigation**: Parallel execution; typically <100ms combined latency; acceptable for <2s response time target
- **Learning Curve**: Team must understand both Postgres and Qdrant
  - **Mitigation**: Well-documented patterns in hybrid-storage.md reference; similar to standard microservices DB patterns
- **Deployment Complexity**: Two services to deploy and monitor
  - **Mitigation**: Docker Compose for local dev; managed services (Neon, Qdrant Cloud) for production
- **Storage Limits**: Neon free tier (512MB) may be tight for large-scale analytics
  - **Mitigation**: Monitor usage; chunk_usage table can be pruned periodically; upgrade to Neon Pro ($19/month) if needed

## Alternatives Considered

### Alternative 1: Postgres with pgvector (Single Database)
**Architecture**: Single Neon Postgres database with pgvector extension for embeddings

**Pros**:
- **Simplicity**: One database, one connection pool, one deployment
- **ACID Guarantees**: All data in single transactional system
- **Easier Backups**: Single database dump for disaster recovery
- **No Synchronization**: Data consistency guaranteed by RDBMS

**Cons**:
- **Slower Vector Search**: pgvector ANN search slower than Qdrant (100-200ms vs 20-50ms at 10K scale)
- **Limited Scaling**: Postgres not optimized for high-dimensional vector operations
- **Resource Contention**: Vector search competes with metadata queries for Postgres compute
- **Index Bloat**: HNSW indexes in pgvector increase database size significantly
- **Rejected**: Performance gap unacceptable for <2s response time target; Qdrant specialization justified

### Alternative 2: Qdrant Only (Vector-First)
**Architecture**: Store all data (metadata + vectors) in Qdrant payload

**Pros**:
- **Simplicity**: Single database, single deployment
- **Fast Vector Search**: Native Qdrant performance
- **Unified Querying**: No synchronization needed

**Cons**:
- **No ACID Transactions**: Qdrant not designed for relational data integrity
- **Complex Joins**: Querying related data (user → queries → chunks → documents) awkward in Qdrant
- **Poor Analytics**: Aggregations, time-series queries (chunk usage over time) not Qdrant's strength
- **BM25 Gap**: No native full-text search (keyword matching) in Qdrant
- **Rejected**: Relational data and analytics requirements make Postgres necessary

### Alternative 3: Pinecone + Postgres
**Architecture**: Postgres for metadata, Pinecone (managed vector DB) for embeddings

**Pros**:
- **Managed Service**: Pinecone fully managed (no Qdrant self-hosting needed)
- **Fast Vector Search**: Performance comparable to Qdrant
- **Scalability**: Pinecone designed for millions of vectors

**Cons**:
- **Cost**: Pinecone free tier very limited (1 index, 1 pod, 1GB); production starts at $70/month
- **Vendor Lock-In**: Proprietary service, migration difficult
- **Less Control**: Cannot self-host or inspect internals
- **Rejected**: Cost prohibitive for educational project; Qdrant open-source provides flexibility

### Alternative 4: Weaviate + Postgres
**Architecture**: Postgres for metadata, Weaviate (vector DB) for embeddings

**Pros**:
- **Open Source**: Self-hostable like Qdrant
- **Integrated BM25**: Weaviate has built-in keyword search (hybrid search native)
- **GraphQL API**: Rich query language

**Cons**:
- **Complexity**: Weaviate schema more complex than Qdrant
- **Resource Heavy**: Higher memory footprint than Qdrant
- **Less Mature**: Smaller ecosystem than Pinecone or Qdrant
- **Rejected**: Qdrant simpler, more performant, better documented for RAG use cases

## References

- Feature Spec: [specs/001-physical-ai-rag-chatbot/spec.md](../../specs/001-physical-ai-rag-chatbot/spec.md)
- Implementation Plan: [specs/001-physical-ai-rag-chatbot/plan.md](../../specs/001-physical-ai-rag-chatbot/plan.md)
- Hybrid Storage Reference: [.claude/skills/physical-ai-rag-builder/references/hybrid-storage.md](../../.claude/skills/physical-ai-rag-builder/references/hybrid-storage.md)
- Related ADRs: ADR-006 (OpenAI Model Selection), ADR-008 (RAG Pipeline Design)

## Validation

- [ ] Benchmark hybrid search (Postgres BM25 + Qdrant vector) vs Postgres-only pgvector on sample queries
- [ ] Measure query latency at 10K chunk scale (target: <100ms combined Postgres + Qdrant)
- [ ] Test synchronization consistency (insert 100 chunks, verify Postgres and Qdrant match)
- [ ] Stress test reconciliation job (introduce inconsistencies, verify detection and correction)
- [ ] Monitor storage usage (Postgres: target <300MB, Qdrant: target <800MB at 10K chunks)
- [ ] Validate atomic operations (simulate Qdrant failure during insert, verify Postgres rollback)
