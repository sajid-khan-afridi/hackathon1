---
id: 002
title: RAG Chatbot Specification Clarifications
stage: misc
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-rag-chatbot
branch: 001-physical-ai-rag-chatbot
user: user
command: /sp.clarify
labels: ["clarification", "spec-refinement", "rag", "chatbot", "openai", "deployment"]
links:
  spec: specs/001-physical-ai-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-rag-chatbot/spec.md
tests:
  - N/A (specification clarification phase)
---

## Prompt

Identify underspecified areas in the RAG chatbot spec and ask targeted clarification questions.

## Focus Areas for Clarification

### 1. Physical AI Book Content

- What is the current state of the book content? (existing markdown files, or needs to be created?)
- Which modules should be prioritized for initial ingestion?
- Are there existing code examples, diagrams, or multimedia to handle?
- What's the expected total content volume? (pages, word count)

### 2. OpenAI Configuration

- Which OpenAI models should we use?
  - Embeddings: text-embedding-ada-002, text-embedding-3-small, or text-embedding-3-large?
  - Generation: GPT-4, GPT-4-turbo, or GPT-3.5-turbo?
- What's the budget/quota for OpenAI API calls?
- Should we implement caching strategies for frequent queries?

### 3. Neon Postgres Setup

- Do you have an existing Neon Postgres instance, or should we create one?
- What's the connection string format? (DATABASE_URL)
- Any specific configuration requirements (connection pooling, SSL)?
- Should we use Neon's serverless features or standard Postgres?

### 4. Qdrant Configuration

- Self-hosted Qdrant or Qdrant Cloud?
- If Cloud: cluster URL and API key available?
- If self-hosted: Docker container or separate server?
- Storage requirements estimation?

### 5. Development Environment

- Team size and development setup (multiple developers or solo)?
- Preferred local development approach:
  - Full Docker Compose (all services containerized)?
  - Hybrid (databases in Docker, apps running locally)?
  - Native installation?
- CI/CD requirements (GitHub Actions, GitLab CI, other)?

### 6. Deployment Strategy

- Where will this be deployed?
  - Frontend: Vercel, Netlify, GitHub Pages, other?
  - Backend: Railway, Render, AWS, GCP, Azure, other?
- Custom domain requirements?
- SSL/HTTPS needed?
- Expected traffic/load?

### 7. Content Ingestion

- How often will book content be updated?
- Should ingestion be manual or automated (on git push)?
- Need for versioning (multiple book versions)?
- Incremental updates vs. full reingestion?

### 8. User Experience

- Should the chat interface be:
  - Embedded in every page?
  - Floating widget (always accessible)?
  - Separate dedicated page?
- Conversation features:
  - Single-turn Q&A or multi-turn conversations?
  - Conversation history (session-based or persistent)?
- Source display preferences:
  - Inline citations in response?
  - Separate "Sources" section?
  - Direct links to book sections?

### 9. Analytics & Monitoring

- What metrics should we track?
  - Popular queries?
  - Retrieval accuracy?
  - User feedback?
  - Performance metrics?
- Logging requirements (structured logging, log aggregation)?
- Error tracking (Sentry, custom)?

### 10. Testing & Validation

- Testing priorities:
  - Focus on backend RAG pipeline?
  - Include frontend E2E tests?
  - Performance/load testing required?
- Test data: use actual book content or sample data?
- Quality metrics for retrieval and generation?

## Prioritization

Ask up to 5 MOST CRITICAL clarifying questions based on the gaps above that would significantly impact the implementation plan.

Focus on:

1. OpenAI model selection (impacts cost and performance)
2. Database setup (Neon/Qdrant existing or new)
3. Content state (existing files or needs creation)
4. Deployment target (affects architecture decisions)
5. Chat UX pattern (embedded vs. floating vs. page)

## Response snapshot

**Clarification Session Completed**: 5 critical questions asked and answered

### Questions & Answers

1. **Physical AI Book Content State**
   - Answer: Existing content in docs/ directory with module-based structure (docs/module-01-ros2/, docs/module-02-gazebo-unity/, docs/module-03-nvidia-isaac/, docs/module-04-vla-conversational/), ready for ingestion

2. **OpenAI Model Selection**
   - Answer: text-embedding-3-small (1536 dimensions) for embeddings + GPT-4-turbo for generation

3. **Database Infrastructure Setup**
   - Answer: Create new local instances via Docker Compose (standard PostgreSQL + Qdrant containers for development, migration path to Neon Postgres for production)

4. **Chat Interface UX Pattern**
   - Answer: Floating widget (always accessible via button, expands to overlay chat panel without disrupting page content)

5. **Deployment Target**
   - Answer: GitHub Pages (frontend static site) + Render (backend FastAPI service with managed Postgres)

### Spec Updates

**Sections Modified**:
- Added `## Clarifications` section with Session 2025-12-06
- Updated `Assumptions` to clarify book content state
- Updated `Dependencies` with specific OpenAI models and database details
- Updated `FR-001` with floating widget specification
- Added `Deployment Strategy` section with GitHub Pages + Render details
- Updated `Deliverables` to reflect floating widget and deployment guides

**Coverage Resolution**:
- Integration & External Dependencies: ✅ Resolved (was Partial)
- Interaction & UX Flow: ✅ Resolved (was Partial)
- Non-Functional Quality Attributes: ✅ Resolved (was Partial)
- Content State: ✅ Resolved (was Missing)
- All other categories: ✅ Clear (unchanged)

### Validation

✅ All 5 questions answered and integrated incrementally
✅ No contradictory statements remain
✅ Terminology normalized (PostgreSQL, floating widget)
✅ Markdown structure valid
✅ Sections coherent and consistent

## Outcome

- ✅ Impact: Resolved 5 critical ambiguities affecting implementation (content source, tech stack, UX pattern, deployment)
- 🧪 Tests: N/A (specification clarification phase)
- 📁 Files: 1 file modified (spec.md with 6 sections updated)
- 🔁 Next prompts: `/sp.plan` to create architectural plan with full context
- 🧠 Reflection: Sequential questioning with immediate integration worked well. All critical decisions documented. Spec now ready for planning with clear technology choices, UX patterns, and deployment targets.

## Evaluation notes (flywheel)

- Failure modes observed: None - all questions answered successfully with clear user choices
- Graders run and results (PASS/FAIL): Coverage analysis PASS (10/10 categories resolved or clear)
- Prompt variant (if applicable): Standard /sp.clarify workflow with sequential questioning
- Next experiment (smallest change to try): Proceed to planning phase with `/sp.plan` - all critical clarifications resolved
