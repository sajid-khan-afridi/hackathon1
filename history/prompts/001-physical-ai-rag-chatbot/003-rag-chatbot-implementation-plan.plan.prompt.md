---
id: 003
title: RAG chatbot implementation plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-rag-chatbot
branch: 001-physical-ai-rag-chatbot
user: uetia
command: /sp.plan
labels: ["planning", "rag", "architecture", "multi-agent"]
links:
  spec: specs/001-physical-ai-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-rag-chatbot/plan.md
tests:
  - None (planning phase, no implementation yet)
---

## Prompt

Create a detailed implementation plan for the RAG chatbot using the physical-ai-rag-builder skill's agent
orchestration.

## Planning Approach

Reference the skill's execution strategy and create a phase-by-phase plan that leverages all 7 agents.

## Expected Plan Structure

### Phase 1: Architecture & Planning (Week 1)

**Agent**: backend-architecture (Plan)
**Duration**: 2-3 days

Deliverables:

- API endpoint specifications (OpenAPI/Swagger)
- Database schema (ERD + SQLAlchemy models design)
- Service layer architecture (RAG pipeline flowchart)
- Pydantic schema definitions
- Authentication/authorization design (if needed)
- Error handling strategy
- Logging and monitoring plan

Technical Decisions:

- OpenAI model selection (embeddings + generation)
- Chunking strategy (token sizes by content type)
- Hybrid search weighting (vector vs. BM25)
- Caching strategy
- Rate limiting approach

Reference: @.claude/skills/physical-ai-rag-builder/references/rag-architecture.md

ADR Candidates:

- ADR-001: OpenAI Model Selection
- ADR-002: Hybrid Storage Architecture
- ADR-003: RAG Pipeline Design

---

### Phase 2: Foundation Setup (Week 1-2)

**Agents**: docusaurus-setup + project-structure-organizer (Parallel)
**Duration**: 3-4 days

#### Agent: docusaurus-setup

Deliverables:

- Docusaurus initialized (if new) or configured (if existing)
- Book content structure (docs/ directory with modules)
- ChatInterface component (React/TypeScript)
- API client service layer
- Theme customization
- Search integration

Reference: @.claude/skills/physical-ai-rag-builder/assets/ChatInterface.tsx.template

#### Agent: project-structure-organizer

Deliverables:

- Monorepo structure (frontend/, backend/, shared/)
- Backend directory structure (app/api, app/services, app/models, tests/)
- Development tooling (Makefile, Docker Compose)
- Environment configuration (.env.example, .gitignore)
- CI/CD workflows (.github/workflows/)
- Documentation (README.md, CONTRIBUTING.md)

Reference: @.claude/skills/physical-ai-rag-builder/references/project-structure.md

Validation:

- Run: scripts/validate_setup.py
- Verify: `make install` completes successfully
- Verify: No file conflicts between frontend/backend

---

### Phase 3: Backend Implementation (Week 2-3)

**Agents**: hybrid-storage-setup → rag-pipeline-builder (Sequential)
**Duration**: 5-7 days

#### Agent: hybrid-storage-setup

Deliverables:

- Neon Postgres setup:

  - SQLAlchemy models (documents, chunks, user_queries, chunk_usage)
  - Alembic migrations
  - Triggers (tsvector auto-update, timestamps)
  - Connection pooling
  - Initialization script (scripts/init_db.py)

- Qdrant setup:

  - Collection creation (physical_ai_chunks)
  - Vector configuration (dim=1536, COSINE distance)
  - Payload indexes (module, chunk_type, learning_level)
  - Client integration (app/services/storage.py)

- Integration:
  - Synchronization strategy
  - Ingestion pipeline (scripts/ingest_documents.py)
  - Consistency guarantees
  - Deletion cascades

Reference: @.claude/skills/physical-ai-rag-builder/references/hybrid-storage.md

Validation:

- Migrations run successfully
- Qdrant collection created
- Sample document ingested
- Hybrid query returns results

#### Agent: rag-pipeline-builder

Deliverables:

- Embeddings service (app/services/embeddings.py):

  - OpenAI client
  - Batch processing
  - Caching
  - Error handling

- Retrieval service (app/services/retrieval.py):

  - Vector search (Qdrant with filters)
  - BM25 search (Postgres full-text)
  - Hybrid scoring (70% vector + 30% BM25)
  - Query classification
  - Reranking

- RAG service (app/services/rag.py):

  - End-to-end pipeline
  - Context assembly
  - Prompt construction
  - Response generation
  - Citation extraction

- Utilities (app/utils/):
  - chunking.py (document chunking)
  - logging.py (structured logging)

Reference: @.claude/skills/physical-ai-rag-builder/references/rag-architecture.md

Validation:

- Unit tests pass (pytest)
- End-to-end RAG query works
- Citations accurate

---

### Phase 4: Integration (Week 3-4)

**Agent**: integration-layer
**Duration**: 3-4 days

Deliverables:

- Backend API routes (app/api/routes/):

  - chat.py (POST /api/v1/chat)
  - health.py (GET /api/v1/health)
  - documents.py (GET /api/v1/documents)

- FastAPI main app (app/main.py):

  - CORS configuration
  - Middleware setup
  - Error handlers
  - OpenAPI documentation

- Frontend API client (src/services/api.ts):

  - TypeScript interfaces
  - chatWithRAG() function
  - Error handling
  - Retry logic

- ChatInterface integration:

  - Connect to API client
  - Loading states
  - Error display
  - Source rendering

- Environment configuration:
  - API URL config (dev vs. prod)
  - static/js/api-config.js

Reference: @.claude/skills/physical-ai-rag-builder/references/project-structure.md

Validation:

- `make dev` runs both servers
- Frontend sends chat request
- Backend returns response with sources
- Sources display correctly
- CORS works without errors

---

### Phase 5: Testing & Validation (Week 4)

**Agent**: testing-validation
**Duration**: 2-3 days

Deliverables:

- Backend tests:

  - Unit tests (services, utils)
  - Integration tests (API endpoints)
  - Performance tests (response time)
  - Coverage report (>80%)

- Frontend tests:

  - Component tests (ChatInterface)
  - API client tests
  - E2E tests (user flow)

- System validation:

  - Run scripts/validate_setup.py
  - Test sample queries (conceptual, code, troubleshooting)
  - Measure retrieval accuracy
  - Measure citation accuracy
  - Performance benchmarks

- Quality metrics:
  - Response time <2s (p95)
  - Retrieval accuracy >85%
  - Citation accuracy 100%
  - Test coverage >80%

Documentation:

- Setup guide
- API documentation
- Deployment guide
- Troubleshooting guide

---

## Dependencies & Critical Path

backend-architecture (Plan)
↓
╔════╩════╗
↓ ↓
docusaurus project-structure
-setup -organizer
↓ ↓
╚════╦════╝
↓
hybrid-storage-setup
↓
rag-pipeline-builder
↓
integration-layer
↓
testing-validation

Critical Path: All agents sequential except Phase 2 (parallel)

## Risk Analysis

1. **OpenAI API Limits**: Implement rate limiting, caching, fallback strategies
2. **Content Volume**: Start with 1 module, scale incrementally
3. **Retrieval Accuracy**: Iterate on chunking and search weights
4. **CORS Issues**: Test early with actual frontend requests
5. **Database Performance**: Monitor query times, add indexes as needed

## Milestones

- M1 (Week 1): Architecture complete, foundation set up
- M2 (Week 2): Databases configured, sample data ingested
- M3 (Week 3): RAG pipeline working, integration complete
- M4 (Week 4): Tests pass, validation complete, ready for deployment

## Success Criteria (Final)

- [ ] All 7 agents completed successfully
- [ ] scripts/validate_setup.py passes
- [ ] `make dev` runs both frontend and backend
- [ ] User can ask question and get response with citations
- [ ] Response time <2s (p95)
- [ ] Test coverage >80%
- [ ] Documentation complete

Reference the skill's agent descriptions for detailed task breakdowns:
@.claude/skills/physical-ai-rag-builder/SKILL.md

## Response snapshot

Created comprehensive implementation plan for Physical AI RAG chatbot system following the physical-ai-rag-builder skill's 7-agent orchestration strategy across 5 phases:

**Phase 0: Research & Prerequisites** (1-2 days)
- Resolve 7 research questions (OpenAI models, chunking, hybrid search, Neon Postgres, Qdrant limits, CORS, Docusaurus widgets)
- Deliverables: research.md, ADR candidates identified

**Phase 1: Architecture & Planning** (2-3 days)
- Agent: backend-architecture (Plan agent)
- Deliverables: data-model.md, contracts/openapi.yaml, quickstart.md, service architecture
- Technical decisions: text-embedding-3-small (1536 dim), GPT-4-turbo, chunking strategy (800/1200/1000 tokens), hybrid search (70% vector + 30% BM25)

**Phase 2: Foundation Setup** (3-4 days, PARALLEL)
- Agent 1: docusaurus-setup → ChatInterface component, API client
- Agent 2: project-structure-organizer → Monorepo structure, Docker Compose, Makefile
- Validation: Structure correct, no conflicts, scripts pass

**Phase 3: Backend Implementation** (5-7 days, SEQUENTIAL)
- Agent 1: hybrid-storage-setup → Postgres + Qdrant configuration, ingestion pipeline
- Agent 2: rag-pipeline-builder → Embeddings, retrieval, RAG services, chunking utilities
- Validation: Migrations run, sample data ingested, E2E query works

**Phase 4: Integration** (3-4 days)
- Agent: integration-layer → Connect frontend to backend, CORS, API routes, error handling
- Validation: `make dev` works, chat functional, sources displayed

**Phase 5: Testing & Validation** (2-3 days)
- Agent: testing-validation → Unit tests, integration tests, E2E tests, performance benchmarks
- Validation: >80% coverage, response time <2s, retrieval accuracy >85%

**Total Duration**: 14-21 days (3-4 weeks)

**Constitution Check**: ✅ PASS - All 7 constitutional principles satisfied
- Content-First Documentation: Existing book structure validated
- Docusaurus Architecture: Autogenerated sidebars confirmed
- GitHub Pages Deployment: Planned for post-v1
- RAG Chatbot Integration: Core requirement
- Hybrid Storage Architecture: Neon Postgres + Qdrant
- Modern AI Stack: FastAPI + OpenAI + streaming
- Test-First for Critical Paths: TDD enforced, >80% coverage

**Project Structure**: Monorepo (frontend/, backend/, shared/) with complete directory trees documented

**Risk Mitigation**: OpenAI API limits (caching), retrieval accuracy (tuning), CORS (early testing), database performance (indexing), content volume (incremental ingestion)

**Success Criteria**: 25 functional requirements, 10 performance targets, all 7 agents complete, validation script passes

## Outcome

- ✅ Impact: Comprehensive implementation plan created for RAG chatbot system spanning 5 phases with 7 specialized agents
- 🧪 Tests: No tests (planning phase only)
- 📁 Files: specs/001-physical-ai-rag-chatbot/plan.md (created, 1116 lines)
- 🔁 Next prompts: /sp.tasks (generate task breakdown from plan), execute Phase 0 research
- 🧠 Reflection: Successfully mapped physical-ai-rag-builder skill's orchestration strategy to concrete implementation plan with clear dependencies, validation criteria, and constitutional compliance

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully
- Graders run and results (PASS/FAIL): Constitution Check PASS (all 7 principles satisfied)
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): Execute Phase 0 research to resolve technical unknowns before proceeding to Phase 1 architecture design
