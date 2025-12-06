# ADR-009: Monorepo Structure for Frontend and Backend

> **Scope**: Repository organization strategy for Docusaurus frontend and FastAPI backend with shared configuration.

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-rag-chatbot
- **Context:** Physical AI book RAG chatbot system

<!-- Significance checklist:
     1) Impact: Yes - Affects development workflow, CI/CD, deployment, and team collaboration
     2) Alternatives: Yes - Monorepo vs separate repos vs backend-serves-frontend
     3) Scope: Yes - Cross-cutting concern affecting all development and deployment processes
-->

## Decision

We will use a **monorepo structure** with separate `frontend/` and `backend/` directories:

**Directory Structure**:
```
physical-ai-book/ (monorepo root)
├── frontend/              # Docusaurus application (Node.js/TypeScript)
│   ├── docs/              # Book content (existing)
│   ├── src/               # React components, services
│   ├── static/            # Static assets
│   └── package.json
├── backend/               # FastAPI application (Python)
│   ├── app/               # Application code
│   ├── tests/             # pytest test suite
│   ├── migrations/        # Alembic migrations
│   └── requirements.txt
├── shared/                # Shared configuration
│   ├── docker/            # Docker Compose, Dockerfiles
│   └── config/            # .env.example, shared configs
├── .github/workflows/     # GitHub Actions CI/CD
├── Makefile               # Unified commands (install, dev, test, clean)
└── README.md              # Single source of truth for setup
```

**Development Workflow**:
- **Single Git Clone**: `git clone` → entire system ready
- **Unified Commands**: `make install`, `make dev`, `make test`, `make clean`
- **Concurrent Development**: `make dev` starts both frontend (port 3000) and backend (port 8000)
- **Atomic Commits**: Frontend + backend changes in single commit (e.g., API contract updates)
- **Shared CI/CD**: Single GitHub Actions workflow tests both layers

**Technology Separation**:
- **Frontend Ecosystem**: npm, Node.js 18+, TypeScript, React, Docusaurus
- **Backend Ecosystem**: pip, Python 3.11+, FastAPI, SQLAlchemy, pytest
- **No Coupling**: Each directory has own dependencies, build process, deployment target

## Consequences

### Positive

- **Single Source of Truth**: Entire system in one repository (no sync issues between repos)
- **Unified Development**: `make dev` starts all services (Postgres, Qdrant, backend, frontend)
- **Atomic Changes**: API contract updates (frontend + backend) in single commit, single PR
- **Simpler CI/CD**: One GitHub Actions workflow for both layers (test frontend → test backend → deploy)
- **Easier Onboarding**: New developers clone one repo, run `make install`, get entire stack
- **Shared Configuration**: `.env.example`, Docker Compose, Makefile centralized in `shared/`
- **Better for Small Teams**: Solo developer or small team benefits from unified workflow
- **Integration Testing**: E2E tests in single repo can test full stack without cross-repo coordination
- **Consistent Versioning**: Frontend and backend versions stay in sync (git tags apply to both)

### Negative

- **Larger Repo Size**: Combined repo larger than separate repos (not a practical issue with Git LFS)
  - **Mitigation**: .gitignore prevents node_modules/, __pycache__/, build artifacts from bloating repo
- **Two Ecosystems**: Developers need both Node.js and Python installed locally
  - **Mitigation**: Docker Compose provides option to run everything in containers (no local installs needed)
- **Deployment Complexity**: Must separate frontend and backend builds in CI/CD
  - **Mitigation**: GitHub Actions supports path filters (`on: push: paths: ["frontend/**"]`)
- **Potential Merge Conflicts**: More activity in single repo could increase conflicts
  - **Mitigation**: Clear directory boundaries (frontend/ vs backend/); conflicts rare in practice
- **CI Build Time**: Testing both layers in single workflow takes longer
  - **Mitigation**: Run frontend and backend tests in parallel (GitHub Actions matrix)

## Alternatives Considered

### Alternative 1: Separate Repositories (frontend repo, backend repo)
**Architecture**: Two independent repos (physical-ai-frontend, physical-ai-backend)

**Pros**:
- **Clear Ownership**: Frontend team owns frontend repo, backend team owns backend repo
- **Independent Deployment**: Deploy frontend without affecting backend (and vice versa)
- **Smaller Repos**: Each repo focused on single technology stack

**Cons**:
- **Synchronization Hell**: API contract changes require coordinating PRs across two repos
- **Duplicate Config**: `.env.example`, Docker Compose duplicated or in separate shared repo
- **Harder Onboarding**: New developers must clone two repos, configure both, understand how they connect
- **Integration Testing**: E2E tests awkward (which repo owns them? How to version sync?)
- **Version Drift**: Frontend and backend versions can drift out of sync
- **Rejected**: Coordination overhead not justified for small team/solo developer

### Alternative 2: Backend Serves Frontend (Single Deployment)
**Architecture**: FastAPI serves Docusaurus build as static files (no separate frontend deployment)

**Pros**:
- **Single Deployment**: One service to deploy, one URL, no CORS needed
- **Simpler Configuration**: No environment-based API URL switching

**Cons**:
- **Docusaurus Not Designed for This**: Docusaurus expects static hosting (GitHub Pages, Vercel)
- **Build Complexity**: Must build Docusaurus, copy to FastAPI static folder, serve from there
- **Deployment Inflexibility**: Cannot deploy frontend to GitHub Pages (violates Constitution Principle III)
- **Development Workflow**: Hot reload broken (Docusaurus dev server separate from FastAPI)
- **Rejected**: Violates constitutional requirement (GitHub Pages hosting)

### Alternative 3: Monorepo with Turborepo/Nx
**Architecture**: Same monorepo structure but with Turborepo or Nx for orchestration

**Pros**:
- **Build Caching**: Turborepo caches builds, tests (faster CI/CD)
- **Dependency Graph**: Nx analyzes dependencies, rebuilds only affected projects
- **Task Pipelines**: Define task execution order declaratively

**Cons**:
- **Overkill for Two Projects**: Turborepo/Nx shine with 5+ packages, not just frontend + backend
- **Learning Curve**: Team must learn Turborepo/Nx conventions
- **Configuration Overhead**: turbo.json or nx.json config files add complexity
- **Makefile Sufficient**: Simple Makefile provides needed commands without extra tooling
- **Rejected**: Unnecessary complexity for two-project monorepo

## References

- Feature Spec: [specs/001-physical-ai-rag-chatbot/spec.md](../../specs/001-physical-ai-rag-chatbot/spec.md)
- Implementation Plan: [specs/001-physical-ai-rag-chatbot/plan.md](../../specs/001-physical-ai-rag-chatbot/plan.md)
- Project Structure Reference: [.claude/skills/physical-ai-rag-builder/references/project-structure.md](../../.claude/skills/physical-ai-rag-builder/references/project-structure.md)
- Related ADRs: ADR-001 (Documentation Platform Stack - Docusaurus + GitHub Pages)

## Validation

- [ ] Verify no file conflicts between frontend/ and backend/ (different .gitignore patterns)
- [ ] Test `make install` succeeds (installs both npm and pip dependencies)
- [ ] Test `make dev` starts all services concurrently (frontend:3000, backend:8000, Postgres, Qdrant)
- [ ] Verify Docker Compose configuration valid (`docker-compose config`)
- [ ] Test CI/CD workflow (GitHub Actions runs tests for both layers)
- [ ] Measure build time (target: <5 minutes for full CI pipeline)
- [ ] Developer onboarding test (new developer clones, runs `make install && make dev`, verifies site works)
