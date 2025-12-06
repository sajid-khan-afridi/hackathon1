---
id: 005
title: Create ADRs for RAG Chatbot Architectural Decisions
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-rag-chatbot
branch: 001-physical-ai-rag-chatbot
user: system
command: /sp.adr
labels: ["architecture", "adr", "rag", "decision-documentation"]
links:
  spec: specs/001-physical-ai-rag-chatbot/spec.md
  ticket: null
  adr: history/adr/006-openai-model-selection-for-rag-pipeline.md
  pr: null
files:
  - history/adr/006-openai-model-selection-for-rag-pipeline.md
  - history/adr/007-hybrid-storage-architecture-postgres-qdrant.md
  - history/adr/008-rag-pipeline-design-weighted-hybrid-search.md
  - history/adr/009-monorepo-structure-frontend-backend.md
  - history/adr/010-variable-size-chunking-strategy.md
tests:
  - No tests run (documentation task)
---

## Prompt

Create Architecture Decision Records for significant decisions in the RAG chatbot implementation.

## ADRs to Create

Based on the plan and the physical-ai-rag-builder skill's architecture, the following decisions are
architecturally significant:

### ADR-001: OpenAI Model Selection for RAG Pipeline
- Embeddings: text-embedding-3-small (1536 dimensions)
- Generation: GPT-4-turbo (128K context)
- Alternatives: ada-002, text-embedding-3-large, GPT-3.5-turbo
- Rationale: Balance of cost and quality

### ADR-002: Hybrid Storage Architecture (Postgres + Qdrant)
- Neon Postgres: Structured metadata, relationships, BM25 search
- Qdrant: Vector embeddings, fast ANN search
- Alternatives: Postgres-only with pgvector, Qdrant-only, Pinecone
- Rationale: Best-of-both-worlds for metadata and vector search

### ADR-003: RAG Pipeline Design with Hybrid Search
- Weighted hybrid search: 70% vector + 30% BM25
- Adaptive weighting based on query classification
- Alternatives: Vector-only, BM25-only, equal weighting, cross-encoder reranking
- Rationale: Superior retrieval accuracy for diverse query types

### ADR-004: Monorepo Structure for Frontend and Backend
- Monorepo with frontend/ and backend/ directories
- Unified development workflow (Makefile)
- Alternatives: Separate repos, backend-serves-frontend
- Rationale: Simplified development, atomic commits, single source of truth

### ADR-005: Chunking Strategy for Physical AI Book Content
- Variable-size chunking: 800/1200/1000 tokens by content type
- 200-token overlap between chunks
- Alternatives: Fixed-size (500 tokens), semantic (by heading), no chunking
- Rationale: Preserves code integrity, optimizes theoretical content precision

## Response snapshot

Created 5 Architecture Decision Records documenting the major architectural decisions for the Physical AI RAG chatbot system:

### ADR-006: OpenAI Model Selection for RAG Pipeline
- **Decision**: text-embedding-3-small (1536-dim) + GPT-4-turbo (128K context)
- **Rationale**: Optimal balance of embedding quality, storage efficiency, and response quality
- **Alternatives Rejected**:
  - ada-002 + GPT-3.5-turbo (lower cost but inadequate quality)
  - text-embedding-3-large (2x storage, marginal quality improvement)
  - Claude 3 Opus (added complexity, no clear benefit)
- **Key Trade-offs**: Higher generation cost justified by educational content quality requirements

### ADR-007: Hybrid Storage Architecture with Postgres and Qdrant
- **Decision**: Neon Postgres (metadata, BM25) + Qdrant (vector search)
- **Rationale**: Leverage Postgres for ACID transactions and complex queries, Qdrant for fast vector similarity
- **Alternatives Rejected**:
  - Postgres-only with pgvector (slower vector search)
  - Qdrant-only (poor for relational data and analytics)
  - Pinecone (cost prohibitive, vendor lock-in)
- **Key Trade-offs**: Two databases increase complexity but deliver superior hybrid search performance

### ADR-008: RAG Pipeline Design with Weighted Hybrid Search
- **Decision**: 70% vector + 30% BM25 with adaptive query classification
- **Rationale**: Hybrid search outperforms single-method retrieval (85% vs 70% accuracy)
- **Alternatives Rejected**:
  - Vector-only (misses exact code terms, error messages)
  - BM25-only (no semantic understanding)
  - Cross-encoder reranking (latency cost not justified for v1)
- **Key Trade-offs**: Increased complexity for significantly better retrieval accuracy

### ADR-009: Monorepo Structure for Frontend and Backend
- **Decision**: Single repository with frontend/ and backend/ directories
- **Rationale**: Unified development workflow, atomic commits, simpler for small teams
- **Alternatives Rejected**:
  - Separate repos (coordination overhead)
  - Backend-serves-frontend (violates GitHub Pages constitutional requirement)
  - Turborepo/Nx (overkill for two projects)
- **Key Trade-offs**: Larger repo size acceptable for development workflow benefits

### ADR-010: Variable-Size Chunking Strategy for Content Types
- **Decision**: Theoretical (800 tokens), Code (1200 tokens), Mixed (1000 tokens), 200-token overlap
- **Rationale**: Content-aware chunking preserves code integrity and optimizes retrieval precision
- **Alternatives Rejected**:
  - Fixed-size 500 tokens (splits code mid-function)
  - Semantic chunking by heading (size variability too extreme: 100-3000 tokens)
  - No chunking (exceeds context window, poor retrieval granularity)
- **Key Trade-offs**: Chunking complexity justified by 85% retrieval accuracy target

All ADRs follow the established template pattern with:
- Clear decision statement
- Comprehensive alternatives analysis
- Explicit consequences (positive and negative)
- Mitigation strategies for negative consequences
- Validation criteria for testing
- References to plan, spec, and skill architecture documents

## Outcome

- ✅ Impact: Created 5 ADRs (006-010) documenting all architecturally significant decisions from plan.md
- 🧪 Tests: No tests (documentation task); ADRs include validation criteria for implementation phases
- 📁 Files: 5 new ADR files in history/adr/
- 🔁 Next prompts: Ready for /sp.tasks to generate task breakdown
- 🧠 Reflection: Successfully identified decision clusters (not atomic choices) per ADR best practices; grouped related technologies (embeddings + generation, Postgres + Qdrant) into cohesive decisions

## Evaluation notes (flywheel)

- Failure modes observed: None; script fallback not needed (created ADRs directly with agent-native tools)
- Graders run and results (PASS/FAIL):
  - ✅ PASS: All decisions are clustered (not over-granular)
  - ✅ PASS: Each ADR lists multiple alternatives with explicit rationale
  - ✅ PASS: Consequences include both positive and negative outcomes with mitigations
  - ✅ PASS: ADRs reference plan.md, spec.md, and skill architecture docs
- Prompt variant (if applicable): Standard /sp.adr workflow
- Next experiment (smallest change to try): None needed; ADR creation successful
