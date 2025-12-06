# Tasks: Physical AI RAG Chatbot System

**Input**: Design documents from `/specs/001-physical-ai-rag-chatbot/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are NOT explicitly requested in the feature specification for v1. Test tasks will focus on validation scripts and end-to-end system testing rather than TDD.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Monorepo structure**: `frontend/`, `backend/`, `shared/` at repository root
- Frontend: Docusaurus site with React components
- Backend: FastAPI service with SQLAlchemy models

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create monorepo directory structure (frontend/, backend/, shared/, .github/workflows/)
- [ ] T002 Initialize backend Python project with FastAPI dependencies in backend/requirements.txt
- [ ] T003 [P] Initialize Alembic for database migrations in backend/migrations/
- [ ] T004 [P] Create Makefile with install, dev, test, build, clean targets
- [ ] T005 [P] Create shared/config/.env.example with all required environment variables
- [ ] T006 [P] Create .gitignore for Python + Node.js + environment files
- [ ] T007 [P] Create Docker Compose configuration in shared/docker/docker-compose.yml (Postgres, Qdrant, backend, frontend services)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T008 Implement SQLAlchemy base models in backend/app/models/database.py (documents, chunks, user_queries, chunk_usage tables)
- [ ] T009 Create Alembic initial migration for database schema in backend/migrations/versions/001_initial_schema.py
- [ ] T010 [P] Implement Pydantic schemas in backend/app/models/schemas.py (ChatRequest, ChatResponse, Citation, HealthResponse)
- [ ] T011 [P] Implement configuration management in backend/app/core/config.py (BaseSettings with env variables)
- [ ] T012 [P] Implement CORS middleware in backend/app/core/security.py
- [ ] T013 Implement database connection setup in backend/app/services/storage.py (async engine, session maker, connection pooling)
- [ ] T014 Create Qdrant client configuration in backend/app/services/storage.py (collection: physical_ai_chunks, 1536 dim, COSINE distance)
- [ ] T015 Create FastAPI main app in backend/app/main.py (CORS config, middleware, startup/shutdown events)

### Frontend Foundation

- [ ] T016 Verify Docusaurus configuration in frontend/docusaurus.config.ts (customFields.apiUrl for environment-based API URL)
- [ ] T017 [P] Create API configuration file in frontend/static/js/api-config.js (window.API_CONFIG)
- [ ] T018 [P] Create TypeScript interfaces for API types in frontend/src/types/api.ts (Message, ChatRequest, ChatResponse, Citation)

### Development Tooling

- [ ] T019 Create database initialization script in backend/scripts/init_db.py (run migrations, create Qdrant collection, verify connections)
- [ ] T020 [P] Create validation script in backend/scripts/validate_setup.py (health checks for all components)
- [ ] T021 [P] Create README.md with setup instructions and architecture overview

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Questions About Physical AI Concepts (Priority: P1) 🎯 MVP

**Goal**: Learners can ask natural language questions and receive accurate answers with citations pointing to relevant book sections.

**Independent Test**: Load sample book content, ask specific topic questions, verify responses include accurate information with proper citations.

### Acceptance Criteria
- Learner asks "What is the difference between a ROS2 publisher and subscriber?" → receives accurate explanation with citations
- Questions about topics in multiple modules → synthesized response with citations to all sources
- Conversational queries → system understands intent and returns relevant answers

### Core RAG Pipeline Implementation

- [ ] T022 [P] [US1] Implement chunking utilities in backend/app/utils/chunking.py (chunk_by_tokens with variable sizes: 800/1200/1000, overlap: 200 tokens)
- [ ] T023 [P] [US1] Implement metadata extraction in backend/app/utils/chunking.py (module, chapter, section from file path and frontmatter)
- [ ] T024 [US1] Implement OpenAI embeddings service in backend/app/services/embeddings.py (generate_embedding, batch_generate_embeddings, caching, retries)
- [ ] T025 [US1] Implement vector search in backend/app/services/retrieval.py (Qdrant search with filters: module, chunk_type, learning_level)
- [ ] T026 [US1] Implement BM25 keyword search in backend/app/services/retrieval.py (Postgres full-text search with ts_rank)
- [ ] T027 [US1] Implement hybrid search scoring in backend/app/services/retrieval.py (70% vector + 30% BM25, normalize scores, deduplicate)
- [ ] T028 [US1] Implement query classification in backend/app/services/retrieval.py (conceptual, code, troubleshooting, comparison patterns)
- [ ] T029 [US1] Implement context assembly in backend/app/services/rag.py (deduplicate chunks, order by relevance, merge overlapping, truncate to 5000 tokens)
- [ ] T030 [US1] Implement response generation in backend/app/services/rag.py (construct prompt with system message + context, call OpenAI GPT-4-turbo, extract citations)
- [ ] T031 [US1] Implement end-to-end RAG service in backend/app/services/rag.py (RAGService.query() method: embed → retrieve → assemble → generate)

### Content Ingestion

- [ ] T032 [US1] Implement document ingestion pipeline in backend/scripts/ingest_documents.py (parse Markdown/MDX, chunk content, generate embeddings, insert into Postgres + Qdrant)
- [ ] T033 [US1] Ingest sample book content from docs/module-01-ros2/ (at least 1 chapter for testing)

### API Endpoints

- [ ] T034 [P] [US1] Implement POST /api/v1/chat endpoint in backend/app/api/routes/chat.py (parse ChatRequest, call RAGService, format ChatResponse with sources)
- [ ] T035 [P] [US1] Implement GET /api/v1/health endpoint in backend/app/api/routes/health.py (check database connection, Qdrant accessibility, API status)
- [ ] T036 [P] [US1] Implement GET /api/v1/documents endpoint in backend/app/api/routes/documents.py (list documents with filtering by module)
- [ ] T037 [US1] Register API routers in backend/app/main.py (chat, health, documents)

### Frontend Chat Interface

- [ ] T038 [P] [US1] Create ChatInterface component in frontend/src/components/ChatInterface/ChatInterface.tsx (floating widget with trigger button + expandable overlay)
- [ ] T039 [P] [US1] Create ChatMessage sub-component in frontend/src/components/ChatInterface/ChatMessage.tsx (user vs assistant message rendering)
- [ ] T040 [P] [US1] Create ChatInterface styles in frontend/src/components/ChatInterface/ChatInterface.module.css (responsive design, mobile-friendly)
- [ ] T041 [US1] Implement API client service in frontend/src/services/api.ts (chatWithRAG function, error handling, retries, TypeScript interfaces)
- [ ] T042 [US1] Connect ChatInterface to API client (call chatWithRAG on message send, handle loading states, display errors)
- [ ] T043 [US1] Implement citation rendering in ChatInterface (display sources with module/chapter/section, clickable links to book sections)

### Error Handling & Logging

- [ ] T044 [P] [US1] Add error handling middleware in backend/app/main.py (validation errors 422, server errors 500, user-friendly messages)
- [ ] T045 [P] [US1] Implement structured logging in backend/app/utils/logging.py (query timestamps, response times, retrieval metrics)
- [ ] T046 [US1] Add query logging to user_queries table in backend/app/services/rag.py (query_text, query_type, response_time, retrieved_chunks)

### Integration & Testing

- [ ] T047 [US1] Test end-to-end flow (start all services with `make dev`, send chat message, verify response with citations)
- [ ] T048 [US1] Validate sample queries (conceptual: "What is ROS2?", code: "Show me a ROS2 publisher example", troubleshooting: "How to fix node not found error?")
- [ ] T049 [US1] Run backend/scripts/validate_setup.py (verify all components healthy)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. Learners can ask questions and get cited responses.

---

## Phase 4: User Story 2 - Browse Book Content with Responsive Interface (Priority: P2)

**Goal**: Learners can access the book from laptop, tablet, or smartphone with proper navigation and responsive chat interface.

**Independent Test**: Load Docusaurus site on different viewport sizes (mobile: 320px-768px, tablet: 768px-1024px, desktop: 1024px+), verify layout and chat functionality.

### Acceptance Criteria
- Book displays correctly on mobile device (viewport 375px) with accessible navigation and chat
- Orientation changes (portrait ↔ landscape) trigger smooth layout adjustments
- Chat interface on desktop allows long questions with proper formatting and citation links

### Responsive Design Implementation

- [ ] T050 [P] [US2] Update ChatInterface.module.css for mobile viewport (320px-768px) in frontend/src/components/ChatInterface/ChatInterface.module.css
- [ ] T051 [P] [US2] Update ChatInterface.module.css for tablet viewport (768px-1024px) in frontend/src/components/ChatInterface/ChatInterface.module.css
- [ ] T052 [P] [US2] Update ChatInterface.module.css for desktop viewport (1024px+) in frontend/src/components/ChatInterface/ChatInterface.module.css
- [ ] T053 [US2] Add touch-friendly interaction elements in ChatInterface (minimum 44x44px tap targets, swipe gestures)
- [ ] T054 [US2] Implement expandable/collapsible chat panel for mobile in ChatInterface.tsx (minimize to icon on small screens)
- [ ] T055 [US2] Add keyboard navigation support in ChatInterface (tab order, enter to send, escape to close)

### Book Content Navigation

- [ ] T056 [US2] Verify module navigation in frontend/sidebars.ts (4 modules: ROS2, Gazebo/Unity, NVIDIA Isaac, VLA models)
- [ ] T057 [US2] Test chapter navigation on mobile (collapsible sidebar, touch-friendly links)
- [ ] T058 [US2] Verify content readability on smallest viewport (320px) with proper text sizing and line breaks

### Cross-Device Testing

- [ ] T059 [US2] Test on mobile browser (Chrome Mobile, Safari iOS) - chat widget opens/closes, messages display
- [ ] T060 [US2] Test on tablet (landscape + portrait) - layout adjusts, navigation accessible
- [ ] T061 [US2] Test on desktop - full features accessible, chat doesn't overlap content

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Book is accessible on all devices with functional chat.

---

## Phase 5: User Story 3 - Get Code Examples and Troubleshooting Help (Priority: P3)

**Goal**: Learners working on practical exercises can ask for code examples or troubleshooting help and receive relevant snippets with syntax highlighting and context.

**Independent Test**: Seed system with code examples, ask code-specific questions (e.g., "Show me how to create a ROS2 publisher in Python"), verify responses include properly formatted code.

### Acceptance Criteria
- Query "How do I create a publisher in ROS2?" → code examples with syntax highlighting and explanations
- Error message description → troubleshooting request with relevant solutions from book
- Comparison question (e.g., "Gazebo vs Unity for robotics simulation?") → structured comparison from multiple modules

### Query Classification Enhancement

- [ ] T062 [US3] Enhance query classification in backend/app/services/retrieval.py (detect code queries with keywords: "how do I", "show me", "example", "code")
- [ ] T063 [US3] Add troubleshooting pattern detection in backend/app/services/retrieval.py (detect error messages, "fix", "error", "not working")
- [ ] T064 [US3] Add comparison query pattern in backend/app/services/retrieval.py (detect "difference", "vs", "compare", "which")
- [ ] T065 [US3] Adjust hybrid search weights based on query type (code queries: favor exact match, conceptual: favor semantic)

### Code Formatting & Display

- [ ] T066 [P] [US3] Implement syntax highlighting for code blocks in ChatMessage.tsx (detect language from markdown code fences)
- [ ] T067 [P] [US3] Add copy-to-clipboard button for code snippets in ChatMessage.tsx
- [ ] T068 [US3] Format code examples with proper indentation and line breaks in RAG response generation (backend/app/services/rag.py)

### Advanced Retrieval

- [ ] T069 [US3] Implement metadata filtering for code chunks in backend/app/services/retrieval.py (filter by chunk_type: code, mixed)
- [ ] T070 [US3] Add reranking for code queries in backend/app/services/retrieval.py (boost chunks containing code blocks)
- [ ] T071 [US3] Implement comparison response format in backend/app/services/rag.py (structure response for "A vs B" queries)

### Testing Code & Troubleshooting Queries

- [ ] T072 [US3] Ingest additional book content with code examples from docs/module-02-gazebo-unity/ and docs/module-03-nvidia-isaac/
- [ ] T073 [US3] Test code queries (e.g., "Show me a ROS2 subscriber example") - verify code snippets with syntax highlighting
- [ ] T074 [US3] Test troubleshooting queries (e.g., "I'm getting 'ImportError: No module named rclpy'") - verify solutions from book
- [ ] T075 [US3] Test comparison queries (e.g., "What's the difference between Gazebo and Unity?") - verify structured comparison

**Checkpoint**: All user stories (US1, US2, US3) should now be independently functional. Learners can ask conceptual questions, browse on any device, and get code help.

---

## Phase 6: User Story 4 - Search Across All Book Content (Priority: P4)

**Goal**: Learners can search for specific technologies or concepts across the entire book using traditional text search and semantic understanding.

**Independent Test**: Search for keywords appearing in multiple modules, verify results include all relevant sections with accurate snippets and relevance ranking.

### Acceptance Criteria
- Search for "ROS2 nodes" → all chapters mentioning nodes, ranked by relevance, with snippet previews
- Search with different terminology ("neural nets" vs "neural networks") → results include semantically similar concepts
- Module filter applied → only results from selected module appear

### Search Infrastructure

- [ ] T076 [P] [US4] Implement GET /api/v1/search endpoint in backend/app/api/routes/search.py (accept query, module filter, return ranked results)
- [ ] T077 [P] [US4] Create SearchRequest and SearchResult schemas in backend/app/models/schemas.py
- [ ] T078 [US4] Implement semantic search function in backend/app/services/retrieval.py (use existing hybrid search, no chat context generation)
- [ ] T079 [US4] Add snippet extraction in backend/app/services/retrieval.py (extract 150-char preview around keyword match)

### Frontend Search UI

- [ ] T080 [P] [US4] Create SearchInterface component in frontend/src/components/SearchInterface/SearchInterface.tsx (search bar + results list)
- [ ] T081 [P] [US4] Create SearchResult component in frontend/src/components/SearchInterface/SearchResult.tsx (display snippet, module, chapter, link)
- [ ] T082 [US4] Add search API client function in frontend/src/services/api.ts (searchContent with module filter)
- [ ] T083 [US4] Integrate SearchInterface into Docusaurus theme (navbar or sidebar)

### Search Optimization

- [ ] T084 [US4] Add module filtering to search in backend/app/services/retrieval.py (filter results by module parameter)
- [ ] T085 [US4] Implement relevance ranking in backend/app/services/retrieval.py (combine vector score + BM25 score + recency)
- [ ] T086 [US4] Add search result caching in backend/app/services/retrieval.py (cache frequent queries for 5 minutes)

### Testing Search

- [ ] T087 [US4] Ingest all remaining book content from docs/module-04-vla-conversational/
- [ ] T088 [US4] Test keyword search (e.g., "ROS2 nodes") - verify all relevant chapters appear
- [ ] T089 [US4] Test semantic search (e.g., "neural nets" vs "neural networks") - verify semantically similar results
- [ ] T090 [US4] Test module filtering - verify only selected module results appear

**Checkpoint**: All user stories (US1-US4) are functional. Learners have full Q&A, responsive UI, code help, and cross-book search.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Documentation

- [ ] T091 [P] Update README.md with complete setup instructions (prerequisites, Docker setup, environment variables, running services)
- [ ] T092 [P] Create deployment guide in docs/DEPLOYMENT.md (GitHub Pages setup, Render configuration, CORS configuration, environment variables)
- [ ] T093 [P] Create troubleshooting guide in docs/TROUBLESHOOTING.md (common issues: CORS errors, database connection, OpenAI API errors)
- [ ] T094 [P] Document API endpoints with examples in docs/API.md (OpenAPI spec reference, sample requests/responses)

### Performance Optimization

- [ ] T095 [P] Add embedding caching to backend/app/services/embeddings.py (in-memory LRU cache with 1000 entry limit)
- [ ] T096 [P] Optimize database queries in backend/app/services/storage.py (add indexes on module, chunk_type, created_at)
- [ ] T097 [P] Add connection pooling configuration in backend/app/services/storage.py (max 10 connections, overflow 20)
- [ ] T098 Benchmark query performance (measure p50, p95, p99 latency for 100 sample queries)

### Security Hardening

- [ ] T099 [P] Implement input sanitization in backend/app/api/routes/chat.py (prevent XSS, SQL injection, command injection)
- [ ] T100 [P] Verify CORS configuration in backend/app/core/security.py (whitelist only allowed origins from env)
- [ ] T101 [P] Add rate limiting monitoring in backend/app/main.py (log excessive requests, prepare for future rate limiting)
- [ ] T102 Audit environment variable usage (ensure no secrets hardcoded, all sensitive config in .env)

### Testing & Validation

- [ ] T103 [P] Write backend unit tests in backend/tests/test_services/test_embeddings.py (test embedding generation, caching)
- [ ] T104 [P] Write backend unit tests in backend/tests/test_services/test_retrieval.py (test hybrid search, query classification)
- [ ] T105 [P] Write backend unit tests in backend/tests/test_services/test_rag.py (test end-to-end pipeline, citation extraction)
- [ ] T106 [P] Write backend unit tests in backend/tests/test_services/test_storage.py (test database operations, Qdrant queries)
- [ ] T107 [P] Write backend integration tests in backend/tests/test_api/test_chat.py (test POST /api/v1/chat with various queries)
- [ ] T108 [P] Write backend integration tests in backend/tests/test_api/test_health.py (test GET /api/v1/health status checks)
- [ ] T109 [P] Write backend integration tests in backend/tests/test_api/test_documents.py (test GET /api/v1/documents filtering)
- [ ] T110 Generate test coverage report (pytest --cov=app --cov-report=html, target >80%)

### System Validation

- [ ] T111 Run backend/scripts/validate_setup.py (verify all components healthy)
- [ ] T112 Validate performance metrics (p95 response time <2s, retrieval accuracy >85%, citation accuracy 100%)
- [ ] T113 Test with all sample query types (conceptual, code, troubleshooting, comparison)
- [ ] T114 Run quickstart validation (another developer follows README to set up and run)

### Final Deliverables

- [ ] T115 Ingest all Physical AI book content (all 4 modules)
- [ ] T116 Create GitHub Actions workflow in .github/workflows/backend-tests.yml (run pytest on push)
- [ ] T117 [P] Create GitHub Actions workflow in .github/workflows/frontend-build.yml (run npm build on push)
- [ ] T118 [P] Create deployment configuration (Render backend config, GitHub Pages build settings)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - Enhances US1 but independently testable
  - User Story 3 (P3): Can start after Foundational - Enhances US1 but independently testable
  - User Story 4 (P4): Depends on US1 completion (reuses retrieval infrastructure) but adds search-specific UI
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### Within Each User Story

#### User Story 1 (P1 - Core Q&A)
- Chunking + Metadata extraction (T022, T023) → Embeddings service (T024)
- Embeddings service (T024) → Vector search (T025)
- Vector search (T025) + BM25 search (T026) → Hybrid search (T027)
- Hybrid search (T027) → Query classification (T028)
- Query classification (T028) → Context assembly (T029)
- Context assembly (T029) → Response generation (T030)
- Response generation (T030) → End-to-end RAG service (T031)
- RAG service (T031) + Ingestion (T032, T033) → API endpoints (T034, T035, T036)
- API endpoints (T034-T037) → Frontend chat interface (T038-T043)
- All above → Error handling (T044-T046) → Integration testing (T047-T049)

#### User Story 2 (P2 - Responsive UI)
- Responsive CSS (T050-T052) can run in parallel
- Touch interactions (T053) → Mobile chat panel (T054)
- Navigation verification (T056-T058) can run in parallel
- Cross-device testing (T059-T061) after all UI updates complete

#### User Story 3 (P3 - Code Examples)
- Query classification enhancement (T062-T064) → Adaptive search weights (T065)
- Syntax highlighting (T066) + Copy button (T067) can run in parallel
- Code formatting (T068) → Metadata filtering (T069) → Reranking (T070)
- Comparison format (T071) → Content ingestion (T072) → Testing (T073-T075)

#### User Story 4 (P4 - Search)
- Search endpoint (T076) + Schemas (T077) can run in parallel
- Semantic search function (T078) → Snippet extraction (T079)
- Frontend SearchInterface (T080) + SearchResult (T081) can run in parallel
- API client (T082) → Integration (T083)
- Search optimization (T084-T086) can run in parallel
- All above → Content ingestion (T087) → Testing (T088-T090)

### Parallel Opportunities

#### Setup Phase (Phase 1)
- T003, T004, T005, T006, T007 can all run in parallel after T002

#### Foundational Phase (Phase 2)
- Backend: T010, T011, T012 can run in parallel after T009
- Frontend: T017, T018 can run in parallel after T016
- Tooling: T020, T021 can run in parallel after T019

#### User Story 1 (Phase 3)
- T022, T023 can run in parallel (different utilities)
- T034, T035, T036 can run in parallel (different API routes)
- T038, T039, T040 can run in parallel (different frontend components)
- T044, T045 can run in parallel (different backend modules)

#### User Story 2 (Phase 4)
- T050, T051, T052 can run in parallel (different CSS files/sections)

#### User Story 3 (Phase 5)
- T066, T067 can run in parallel (different ChatMessage features)

#### User Story 4 (Phase 6)
- T076, T077 can run in parallel (backend endpoint + schemas)
- T080, T081 can run in parallel (different frontend components)

#### Polish Phase (Phase 7)
- All documentation tasks (T091-T094) can run in parallel
- All performance tasks (T095-T098) can run in parallel
- All security tasks (T099-T102) can run in parallel
- All test writing tasks (T103-T109) can run in parallel
- T116, T117, T118 can run in parallel

---

## Parallel Example: User Story 1 Core Components

```bash
# Launch chunking utilities together:
Task: "Implement chunking utilities in backend/app/utils/chunking.py"
Task: "Implement metadata extraction in backend/app/utils/chunking.py"

# Launch API routes together (different files):
Task: "Implement POST /api/v1/chat endpoint in backend/app/api/routes/chat.py"
Task: "Implement GET /api/v1/health endpoint in backend/app/api/routes/health.py"
Task: "Implement GET /api/v1/documents endpoint in backend/app/api/routes/documents.py"

# Launch frontend components together:
Task: "Create ChatInterface component in frontend/src/components/ChatInterface/ChatInterface.tsx"
Task: "Create ChatMessage sub-component in frontend/src/components/ChatInterface/ChatMessage.tsx"
Task: "Create ChatInterface styles in frontend/src/components/ChatInterface/ChatInterface.module.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Core Q&A)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T021) - CRITICAL GATE
3. Complete Phase 3: User Story 1 (T022-T049)
4. **STOP and VALIDATE**: Test User Story 1 independently
   - Can learners ask questions and get cited responses?
   - Do citations link to correct book sections?
   - Is response time <2s (p95)?
5. Deploy/demo if ready

### Incremental Delivery (Add Features Progressively)

1. Setup + Foundational → Foundation ready (T001-T021)
2. Add User Story 1 → Test independently → Deploy/Demo (MVP achieved!) (T022-T049)
3. Add User Story 2 → Test independently → Deploy/Demo (responsive UI added) (T050-T061)
4. Add User Story 3 → Test independently → Deploy/Demo (code help added) (T062-T075)
5. Add User Story 4 → Test independently → Deploy/Demo (search added) (T076-T090)
6. Polish → Final production release (T091-T118)

Each story adds value without breaking previous stories.

### Parallel Team Strategy (If Multiple Developers Available)

With 3+ developers:

1. **Team completes Setup + Foundational together** (T001-T021)
2. **Once Foundational is done**, split:
   - **Developer A**: User Story 1 (T022-T049) - Core Q&A
   - **Developer B**: User Story 2 (T050-T061) - Responsive UI (can start early, mostly frontend)
   - **Developer C**: User Story 3 (T062-T075) - Code examples (builds on US1, coordinate integration)
3. Stories complete and integrate independently
4. **Team reconvenes** for User Story 4 (T076-T090) - Search (builds on US1 retrieval)
5. **Team** completes Polish together (T091-T118)

---

## Notes

- **[P] tasks** = different files, no dependencies, safe to run in parallel
- **[Story] label** maps task to specific user story for traceability (US1, US2, US3, US4)
- Each user story should be **independently completable and testable**
- **Commit after each task** or logical group for rollback safety
- **Stop at any checkpoint** to validate story independently before proceeding
- **Avoid**: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Success Criteria Mapping

### Functional Requirements (from spec.md)

- **FR-001 to FR-025**: All covered across User Stories 1-4
  - FR-001 (floating widget): T038 (ChatInterface component)
  - FR-002 (500 char limit): T034 (chat endpoint validation)
  - FR-003 (query classification): T028, T062-T064
  - FR-004 (hybrid search): T027
  - FR-005 (book-grounded responses): T030 (response generation)
  - FR-006, FR-007 (citations): T030, T043
  - FR-008 (intelligent chunking): T022
  - FR-009 (metadata storage): T008 (SQLAlchemy models)
  - FR-010 (vector embeddings): T014 (Qdrant)
  - FR-011 (synchronization): T032 (ingestion pipeline)
  - FR-012, FR-013 (health/docs endpoints): T035, T036
  - FR-014 (input validation): T010 (Pydantic schemas)
  - FR-015 (error handling): T044
  - FR-016, FR-017 (logging/analytics): T045, T046
  - FR-018 (4 modules): T056 (navigation)
  - FR-019 (responsive): T050-T055 (User Story 2)
  - FR-020 (real-time display): T042 (loading states)
  - FR-021 (deduplication): T029
  - FR-022 (caching): T024, T095
  - FR-023 (env-based config): T005, T011
  - FR-024 (containerized dev): T007 (Docker Compose)
  - FR-025 (single command): T004 (Makefile)

### Success Criteria (from spec.md)

- **SC-001 (p95 <2s)**: T098 (performance benchmarking), T112 (validation)
- **SC-002 (85% accuracy)**: T048, T112 (sample query testing)
- **SC-003 (100% citations)**: T043, T112
- **SC-004 (10+ concurrent)**: T097 (connection pooling)
- **SC-005 (mobile 375px)**: T059 (mobile testing)
- **SC-006 (90% first attempt)**: T112 (query success rate)
- **SC-007 (80% coverage)**: T110 (coverage report)
- **SC-008 (services start <30s)**: T111 (validate_setup.py)
- **SC-009 (zero critical vulns)**: T099-T102 (security hardening)
- **SC-010 (validation healthy)**: T111 (system validation)

---

**Generated**: 2025-12-06
**Total Tasks**: 118
**Phases**: 7 (Setup, Foundational, US1, US2, US3, US4, Polish)
**Critical Path**: Setup → Foundational → User Story 1 (MVP)
