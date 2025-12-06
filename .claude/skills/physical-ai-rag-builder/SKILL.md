---
name: physical-ai-rag-builder
description: Comprehensive skill for building RAG chatbots with FastAPI, OpenAI, and hybrid storage (Neon Postgres + Qdrant) for the Physical AI book project with Docusaurus. Use when users need to (1) Build RAG chatbot systems, (2) Integrate FastAPI backends with Docusaurus frontends, (3) Set up hybrid vector storage with Postgres and Qdrant, (4) Implement document retrieval with embeddings, (5) Create educational content platforms with AI assistants, or (6) Bootstrap Physical AI book projects with integrated Q&A capabilities.
---

# Physical AI RAG Builder

Build production-ready RAG chatbots for the Physical AI book project using FastAPI, OpenAI, Neon Postgres, Qdrant, and Docusaurus.

## Quick Start

When this skill is invoked, follow this workflow:

1. **Understand requirements** - Clarify user's specific needs
2. **Plan execution** - Determine which agents to run and in what order
3. **Execute agents** - Launch agents sequentially or in parallel
4. **Validate outputs** - Ensure each agent produces expected results
5. **Integrate components** - Verify smooth integration between frontend and backend

## Agent Orchestration

This skill coordinates **7 specialized agents** to build the complete RAG system:

### Agent 1: `backend-architecture` (Plan Agent)
**Purpose**: Design the FastAPI backend architecture

**When to run**: Always run first to establish architectural decisions

**Inputs**:
- User requirements for RAG functionality
- Physical AI book structure (modules, chapters)
- Integration requirements (Docusaurus, storage)

**Outputs**:
- API endpoint design (routes, schemas)
- Service layer architecture (RAG pipeline, storage)
- Database schema design
- Authentication/authorization plan
- Error handling strategy

**Task description for agent**:
```
Design a FastAPI backend architecture for a RAG chatbot serving the Physical AI book.

Requirements:
- RESTful API with /api/v1/chat endpoint
- Hybrid storage: Neon Postgres (metadata) + Qdrant (vectors)
- OpenAI embeddings (ada-002 or text-embedding-3)
- Pydantic schemas for request/response validation
- CORS configuration for Docusaurus frontend
- Health check and analytics endpoints

Reference: See references/rag-architecture.md for RAG patterns

Produce:
1. API endpoint specifications (OpenAPI/Swagger)
2. Database schema (SQLAlchemy models)
3. Service layer design (RAG pipeline flow)
4. Configuration management approach
5. Testing strategy
```

**Validation checklist**:
- [ ] API endpoints documented with request/response schemas
- [ ] Database schema includes all required tables (documents, chunks, user_queries)
- [ ] RAG pipeline flow clearly defined
- [ ] CORS and security considerations addressed
- [ ] Error handling and logging strategy defined

---

### Agent 2: `docusaurus-setup` (General-Purpose Agent)
**Purpose**: Bootstrap or configure the Docusaurus site for the Physical AI book

**When to run**: Run in parallel with `project-structure-organizer` (Phase 2)

**Inputs**:
- Existing Docusaurus site (if any)
- Book structure (modules, chapters, sections)
- Frontend requirements (chat interface, search)

**Outputs**:
- Initialized Docusaurus project (if new)
- Configured theme and plugins
- Content structure (docs/ directory)
- ChatInterface component integrated
- API client configured

**Task description for agent**:
```
Set up Docusaurus site for Physical AI book with integrated chat interface.

If Docusaurus doesn't exist:
1. Initialize Docusaurus project in frontend/ directory
2. Configure for technical documentation (code highlighting, math support)
3. Set up module-based content structure

Always:
4. Create ChatInterface component (use assets/ChatInterface.tsx.template)
5. Configure API client to connect to FastAPI backend
6. Add custom CSS for chat interface
7. Configure sidebars for book navigation
8. Set up search integration

Reference: assets/ChatInterface.tsx.template for component structure

Produce:
- Working Docusaurus site (npm start should work)
- ChatInterface component in src/components/
- API service layer in src/services/api.ts
- Updated docusaurus.config.ts with custom fields (apiUrl)
```

**Validation checklist**:
- [ ] Docusaurus site runs successfully (npm start)
- [ ] ChatInterface component renders without errors
- [ ] API client configured with correct backend URL
- [ ] Content structure matches book modules
- [ ] Build succeeds (npm run build)

---

### Agent 3: `project-structure-organizer` (General-Purpose Agent)
**Purpose**: Organize backend and frontend file structure for seamless integration

**When to run**: Run in parallel with `docusaurus-setup` (Phase 2)

**Inputs**:
- Project root directory
- Architecture decisions from backend-architecture agent
- Frontend setup from docusaurus-setup agent

**Outputs**:
- Complete monorepo structure (frontend/ + backend/)
- Shared configuration (docker/, .env)
- Development tooling (Makefile, scripts)
- .gitignore and documentation

**Task description for agent**:
```
Organize the complete project structure for Physical AI RAG system.

Use reference: references/project-structure.md for recommended structure

Tasks:
1. Create monorepo structure:
   - frontend/ (Docusaurus)
   - backend/ (FastAPI)
   - shared/ (Docker, config)
   - .github/ (CI/CD)

2. Set up backend structure:
   - app/api/routes/ (chat.py, documents.py, health.py)
   - app/services/ (rag.py, embeddings.py, retrieval.py, storage.py)
   - app/models/ (database.py, schemas.py)
   - app/core/ (config.py, security.py)
   - tests/, scripts/, migrations/

3. Create development tooling:
   - Makefile with install/dev/test/build commands
   - Docker Compose (use assets/docker-compose.yml.template)
   - .env.example with all required variables
   - .gitignore for Python + Node.js

4. Ensure no file/directory conflicts between frontend and backend

Validation script: scripts/validate_setup.py

Produce:
- Complete directory structure
- All placeholder files created
- Development tooling configured
- Documentation (README.md)
```

**Validation checklist**:
- [ ] Monorepo structure created correctly
- [ ] Backend directory structure matches architecture
- [ ] Frontend directory structure from docusaurus-setup preserved
- [ ] Makefile commands work (make install, make dev)
- [ ] Docker Compose configuration valid
- [ ] No file conflicts between frontend/backend
- [ ] scripts/validate_setup.py passes

---

### Agent 4: `hybrid-storage-setup` (General-Purpose Agent)
**Purpose**: Configure Neon Postgres and Qdrant for hybrid vector storage

**When to run**: After project-structure-organizer (Phase 3)

**Inputs**:
- Database schema from backend-architecture
- Project structure from project-structure-organizer
- Neon Postgres connection string
- Qdrant URL and API key

**Outputs**:
- SQLAlchemy models implemented
- Qdrant collection configured
- Database migrations (Alembic)
- Connection pooling configured
- Test data ingestion script

**Task description for agent**:
```
Set up hybrid storage system with Neon Postgres and Qdrant.

Reference: references/hybrid-storage.md for complete schema and patterns

Phase 1: Neon Postgres Setup
1. Implement SQLAlchemy models (documents, chunks, user_queries, chunk_usage)
2. Create Alembic migrations
3. Add triggers for auto-updating tsvector and timestamps
4. Configure connection pooling
5. Create initialization script (scripts/init_db.py)

Phase 2: Qdrant Setup
1. Create Qdrant collection (physical_ai_chunks)
2. Configure vector params (size=1536, distance=COSINE)
3. Set up payload indexes (module, chunk_type, learning_level)
4. Implement connection client in app/services/storage.py

Phase 3: Integration
5. Implement synchronization strategy
6. Create ingestion pipeline (scripts/ingest_documents.py)
7. Add reconciliation job for consistency
8. Implement hybrid query patterns

Test:
- Run migrations successfully
- Create Qdrant collection
- Ingest sample document
- Query both databases successfully

Produce:
- app/models/database.py (SQLAlchemy models)
- migrations/versions/*.py (Alembic migrations)
- app/services/storage.py (DB operations)
- scripts/init_db.py (database initialization)
- scripts/ingest_documents.py (data ingestion)
```

**Validation checklist**:
- [ ] Database migrations run successfully
- [ ] All tables created with correct schema
- [ ] Qdrant collection created and accessible
- [ ] Payload indexes configured
- [ ] Connection pooling works
- [ ] Sample document ingested successfully
- [ ] Hybrid query returns results

---

### Agent 5: `rag-pipeline-builder` (General-Purpose Agent)
**Purpose**: Implement the core RAG pipeline (retrieval + generation)

**When to run**: After hybrid-storage-setup (Phase 3)

**Inputs**:
- Storage layer from hybrid-storage-setup
- Architecture design from backend-architecture
- OpenAI API key

**Outputs**:
- Embeddings service (OpenAI integration)
- Retrieval service (hybrid search)
- RAG service (end-to-end pipeline)
- Chunking utilities
- Response generation with citations

**Task description for agent**:
```
Implement the complete RAG pipeline for Physical AI book queries.

Reference: references/rag-architecture.md for pipeline design

Phase 1: Embeddings Service (app/services/embeddings.py)
1. OpenAI client configuration
2. Batch embedding generation
3. Error handling and retries
4. Caching strategy

Phase 2: Retrieval Service (app/services/retrieval.py)
5. Vector search in Qdrant (with filters)
6. BM25 search in Postgres (full-text)
7. Hybrid scoring and reranking
8. Query classification (conceptual, code, troubleshooting)
9. Metadata filtering (module, learning_level)

Phase 3: RAG Service (app/services/rag.py)
10. Query processing and embedding
11. Context retrieval (hybrid search)
12. Context assembly and deduplication
13. Prompt construction with Physical AI context
14. Response generation with OpenAI
15. Citation extraction and formatting

Phase 4: Utilities (app/utils/chunking.py)
16. Document chunking (variable sizes based on type)
17. Metadata extraction
18. Preprocessing and cleaning

Test:
- Generate embeddings for sample queries
- Retrieve relevant chunks
- Generate complete response with citations
- Verify citation accuracy

Produce:
- app/services/embeddings.py
- app/services/retrieval.py
- app/services/rag.py
- app/utils/chunking.py
- Unit tests in tests/test_services/
```

**Validation checklist**:
- [ ] OpenAI embeddings generation works
- [ ] Hybrid search returns relevant results
- [ ] Query classification accurate
- [ ] Context assembly produces coherent context
- [ ] Response generation includes citations
- [ ] Citations link to correct book sections
- [ ] Error handling for API failures
- [ ] Unit tests pass

---

### Agent 6: `integration-layer` (General-Purpose Agent)
**Purpose**: Connect Docusaurus frontend to FastAPI backend

**When to run**: After rag-pipeline-builder (Phase 4)

**Inputs**:
- Completed backend with RAG pipeline
- Docusaurus site from docusaurus-setup
- ChatInterface component

**Outputs**:
- API routes implemented (FastAPI)
- API client integrated (Docusaurus)
- CORS configured
- Error handling on both sides
- Loading states and UI feedback

**Task description for agent**:
```
Integrate the Docusaurus frontend with FastAPI backend.

Reference:
- references/project-structure.md for integration patterns
- assets/main.py.template for backend structure

Backend Tasks (FastAPI):
1. Implement API routes in app/api/routes/chat.py
   - POST /api/v1/chat (main chat endpoint)
   - GET /api/v1/health (health check)
   - GET /api/v1/documents (list documents)

2. Wire up RAG service to chat endpoint
   - Parse ChatRequest
   - Call rag_service.generate_response()
   - Format ChatResponse with sources

3. Configure CORS in app/main.py
   - Allow frontend origin (http://localhost:3000)
   - Allow credentials
   - Set proper headers

4. Add error handling middleware
   - Validation errors (422)
   - Server errors (500)
   - Rate limiting (429)

Frontend Tasks (Docusaurus):
5. Implement API client in src/services/api.ts
   - chatWithRAG() function
   - Error handling and retries
   - TypeScript interfaces

6. Connect ChatInterface to API client
   - Call API on message send
   - Handle loading states
   - Display errors gracefully
   - Show sources with citations

7. Environment configuration
   - Use window.API_CONFIG for API URL
   - Support dev and prod environments
   - Create static/js/api-config.js

Test Integration:
8. Start both servers (make dev)
9. Send chat message from frontend
10. Verify backend receives request
11. Verify response displayed correctly
12. Verify sources shown

Produce:
- backend/app/api/routes/chat.py (complete)
- backend/app/main.py (CORS configured)
- frontend/src/services/api.ts (complete)
- frontend/src/components/ChatInterface/ChatInterface.tsx (connected)
- Integration test demonstrating end-to-end flow
```

**Validation checklist**:
- [ ] Backend API endpoints respond correctly
- [ ] CORS allows frontend requests
- [ ] Frontend can send chat requests
- [ ] Responses display in chat interface
- [ ] Sources shown with module/chapter links
- [ ] Errors handled gracefully
- [ ] Loading states work
- [ ] Both dev and prod configurations work

---

### Agent 7: `testing-validation` (General-Purpose Agent)
**Purpose**: Comprehensive testing and validation of the entire system

**When to run**: Last (Phase 5), after integration-layer

**Inputs**:
- Complete integrated system
- All previous agent outputs

**Outputs**:
- Unit tests for backend services
- Integration tests for API endpoints
- E2E tests for frontend-backend flow
- Performance benchmarks
- Validation report

**Task description for agent**:
```
Comprehensive testing and validation of Physical AI RAG system.

Backend Testing:
1. Unit tests for services
   - test_embeddings.py (embedding generation)
   - test_retrieval.py (hybrid search)
   - test_rag.py (end-to-end pipeline)
   - test_storage.py (database operations)

2. Integration tests for API
   - test_chat_endpoint.py
   - test_health_endpoint.py
   - test_error_handling.py

3. Performance tests
   - Query latency (<2s p95)
   - Embedding generation time
   - Database query performance
   - Memory usage

Frontend Testing:
4. Component tests
   - ChatInterface rendering
   - Message display
   - API error handling

5. E2E tests
   - User sends message → receives response
   - Sources displayed correctly
   - Error states shown

System Validation:
6. Run validation script: python scripts/validate_setup.py
7. Verify all components healthy
8. Test with sample queries from each category:
   - Conceptual: "What is ROS2?"
   - Code: "Show me a ROS2 publisher example"
   - Troubleshooting: "How to fix node not found error?"

9. Measure quality metrics:
   - Retrieval accuracy (relevant chunks retrieved)
   - Citation accuracy (sources match content)
   - Response quality (coherent, helpful)

10. Load testing (optional)
    - Concurrent user simulation
    - Database connection pooling
    - Response time under load

Produce:
- Complete test suite in backend/tests/
- Test coverage report (>80%)
- Performance benchmark results
- Validation report with metrics
- List of any issues found
```

**Validation checklist**:
- [ ] All unit tests pass
- [ ] Integration tests pass
- [ ] E2E tests demonstrate full flow
- [ ] Performance meets targets (<2s response time)
- [ ] Test coverage >80%
- [ ] Validation script passes
- [ ] Sample queries produce good results
- [ ] No critical issues found

---

## Execution Strategy

### Phase-Based Execution

```
Phase 1: Planning (Sequential)
└─► backend-architecture (Plan agent)

Phase 2: Foundation (Parallel)
├─► docusaurus-setup (General-purpose)
└─► project-structure-organizer (General-purpose)

Phase 3: Backend Implementation (Sequential)
├─► hybrid-storage-setup (General-purpose)
└─► rag-pipeline-builder (General-purpose)

Phase 4: Integration (Sequential)
└─► integration-layer (General-purpose)

Phase 5: Validation (Sequential)
└─► testing-validation (General-purpose)
```

### Conditional Execution

- **Skip docusaurus-setup** if Docusaurus site already exists
- **Skip hybrid-storage-setup** if databases already configured
- **Skip testing-validation** if user requests quick prototype

### Error Handling

- If agent fails: Retry up to 3 times with exponential backoff
- If agent blocked: Request user clarification
- If validation fails: Roll back agent changes and retry
- Track state in .claude/state/execution-state.json

## Reference Files

Detailed information available in reference files:

- **rag-architecture.md** - RAG pipeline patterns, chunking, retrieval, generation
- **hybrid-storage.md** - Database schemas, Qdrant config, query patterns
- **agent-coordination.md** - Communication protocols, execution patterns, state management
- **project-structure.md** - Directory structure, integration patterns, deployment

Load these files as needed when agents require detailed guidance.

## Scripts

Utility scripts for common operations:

- **scripts/init_project_structure.py** - Bootstrap complete project structure
- **scripts/validate_setup.py** - Validate all components are configured correctly

Run these scripts to accelerate setup and validation.

## Assets

Template files for key components:

- **assets/ChatInterface.tsx.template** - React component for chat interface
- **assets/main.py.template** - FastAPI application structure
- **assets/docker-compose.yml.template** - Docker Compose for local development

Copy and customize these templates during agent execution.

## Best Practices

1. **Always validate** after each agent completes
2. **Use checkpoints** to enable resuming from failures
3. **Parallel execution** for independent agents (docusaurus-setup + project-structure-organizer)
4. **Sequential execution** for dependent agents (storage → RAG → integration)
5. **User involvement** for architectural decisions (AskUserQuestion)
6. **Incremental testing** as components are built
7. **Clear handoffs** between agents with validated outputs

## Common Issues and Solutions

**Issue**: CORS errors when frontend calls backend
- **Solution**: Verify ALLOWED_ORIGINS in backend .env includes frontend URL

**Issue**: Database connection fails
- **Solution**: Check DATABASE_URL format, ensure Docker containers running

**Issue**: Qdrant collection not found
- **Solution**: Run scripts/init_db.py to create collection

**Issue**: OpenAI API errors
- **Solution**: Verify OPENAI_API_KEY is set, check API quota

**Issue**: No results from retrieval
- **Solution**: Ensure documents ingested (run scripts/ingest_documents.py)

**Issue**: Frontend build fails
- **Solution**: Check node_modules installed (npm install), verify TypeScript config

## Success Criteria

The Physical AI RAG system is complete when:

- [x] User can access Docusaurus site (http://localhost:3000)
- [x] User can type question in chat interface
- [x] Backend retrieves relevant context from hybrid storage
- [x] OpenAI generates response with citations
- [x] Response displays in chat with source links
- [x] Sources link to correct book sections
- [x] System handles errors gracefully
- [x] Performance meets targets (<2s response time)
- [x] All tests pass
- [x] Validation script reports healthy

## Next Steps After Completion

1. **Content Ingestion**: Ingest all Physical AI book chapters
2. **Fine-tuning**: Adjust chunking, retrieval weights, prompt templates
3. **Analytics**: Add usage tracking, popular queries, feedback collection
4. **Optimization**: Cache frequent queries, optimize database indexes
5. **Deployment**: Deploy to production (Vercel + Railway/Render)
6. **Monitoring**: Set up logging, error tracking (Sentry), uptime monitoring
