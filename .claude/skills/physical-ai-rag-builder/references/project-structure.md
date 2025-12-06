# Project Structure: Frontend + Backend Integration

## Table of Contents
- [Recommended Directory Structure](#recommended-directory-structure)
- [Integration Patterns](#integration-patterns)
- [File Organization](#file-organization)
- [Development Workflow](#development-workflow)
- [Deployment Considerations](#deployment-considerations)

## Recommended Directory Structure

### Monorepo Structure (Recommended for Physical AI Book)

```
physical-ai-book/
├── frontend/                      # Docusaurus application
│   ├── docs/                      # Documentation content
│   │   ├── module-01-ros2/
│   │   ├── module-02-gazebo-unity/
│   │   ├── module-03-nvidia-isaac/
│   │   └── module-04-vla-conversational/
│   ├── src/
│   │   ├── components/            # React components
│   │   │   ├── ChatInterface/     # RAG chatbot UI
│   │   │   │   ├── ChatInterface.tsx
│   │   │   │   ├── ChatMessage.tsx
│   │   │   │   └── ChatInterface.module.css
│   │   │   └── CodeBlock/
│   │   ├── pages/
│   │   └── theme/                 # Theme customizations
│   ├── static/                    # Static assets
│   ├── docusaurus.config.ts       # Docusaurus configuration
│   ├── sidebars.ts
│   ├── package.json
│   └── tsconfig.json
│
├── backend/                       # FastAPI application
│   ├── app/
│   │   ├── __init__.py
│   │   ├── main.py                # FastAPI app entry
│   │   ├── api/
│   │   │   ├── __init__.py
│   │   │   ├── routes/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── chat.py        # Chat endpoints
│   │   │   │   ├── documents.py   # Document management
│   │   │   │   └── health.py      # Health checks
│   │   │   └── deps.py            # Dependencies
│   │   ├── core/
│   │   │   ├── __init__.py
│   │   │   ├── config.py          # Settings
│   │   │   └── security.py        # Auth
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   ├── database.py        # SQLAlchemy models
│   │   │   └── schemas.py         # Pydantic schemas
│   │   ├── services/
│   │   │   ├── __init__.py
│   │   │   ├── rag.py             # RAG pipeline
│   │   │   ├── embeddings.py      # OpenAI embeddings
│   │   │   ├── retrieval.py       # Hybrid search
│   │   │   └── storage.py         # DB operations
│   │   └── utils/
│   │       ├── __init__.py
│   │       ├── chunking.py        # Document chunking
│   │       └── logging.py         # Logging setup
│   ├── migrations/                # Alembic migrations
│   │   ├── versions/
│   │   └── env.py
│   ├── tests/
│   │   ├── __init__.py
│   │   ├── conftest.py
│   │   ├── test_api/
│   │   └── test_services/
│   ├── scripts/
│   │   ├── ingest_documents.py    # Data ingestion
│   │   └── init_db.py             # Database setup
│   ├── requirements.txt
│   ├── requirements-dev.txt
│   └── pyproject.toml
│
├── shared/                        # Shared configurations
│   ├── docker/
│   │   ├── docker-compose.yml
│   │   ├── Dockerfile.backend
│   │   └── Dockerfile.frontend
│   └── config/
│       └── .env.example
│
├── .github/                       # CI/CD
│   └── workflows/
│       ├── backend-tests.yml
│       └── frontend-build.yml
│
├── docs-raw/                      # Source documentation (markdown)
│   └── (raw content before processing)
│
├── .gitignore
├── README.md
└── Makefile                       # Common commands
```

### Alternative: Separate Repos Structure

```
physical-ai-frontend/              # Frontend repository
├── (Docusaurus structure as above)
└── .env.production               # Backend API URL

physical-ai-backend/               # Backend repository
├── (FastAPI structure as above)
└── .env.production               # CORS allowed origins
```

## Integration Patterns

### Pattern 1: Monorepo with Shared Build

**Advantages:**
- Single source of truth
- Easier local development
- Shared CI/CD pipeline
- Atomic commits across frontend/backend

**Configuration:**

```makefile
# Makefile for unified development
.PHONY: install dev build test clean

install:
	cd frontend && npm install
	cd backend && pip install -r requirements.txt

dev:
	# Run both frontend and backend concurrently
	make -j2 dev-frontend dev-backend

dev-frontend:
	cd frontend && npm start

dev-backend:
	cd backend && uvicorn app.main:app --reload --port 8000

build:
	cd frontend && npm run build
	# Backend doesn't need build step (Python)

test:
	cd frontend && npm test
	cd backend && pytest

clean:
	cd frontend && rm -rf node_modules build .docusaurus
	cd backend && rm -rf __pycache__ .pytest_cache
```

### Pattern 2: API Proxy in Development

**Docusaurus dev server configuration:**

```typescript
// frontend/docusaurus.config.ts
import type {Config} from '@docusaurus/types';

const config: Config = {
  // ... other config

  // Proxy API requests to backend in development
  plugins: [
    // ... other plugins
    [
      '@docusaurus/plugin-client-redirects',
      {
        // Proxy /api/* to backend
      },
    ],
  ],

  scripts: [
    {
      src: '/js/api-config.js',
      async: false,
    },
  ],

  customFields: {
    apiUrl: process.env.API_URL || 'http://localhost:8000',
  },
};

export default config;
```

**Static API configuration:**

```javascript
// frontend/static/js/api-config.js
window.API_CONFIG = {
  baseUrl: process.env.NODE_ENV === 'production'
    ? 'https://api.physical-ai.com'
    : 'http://localhost:8000',
  timeout: 30000,
};
```

### Pattern 3: CORS Configuration

**Backend CORS setup:**

```python
# backend/app/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.core.config import settings

app = FastAPI(title="Physical AI RAG API")

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,  # ["http://localhost:3000", "https://physical-ai.com"]
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE"],
    allow_headers=["*"],
)
```

**Environment-based configuration:**

```python
# backend/app/core/config.py
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # API
    API_V1_STR: str = "/api/v1"

    # CORS
    ALLOWED_ORIGINS: list[str] = ["http://localhost:3000"]

    # Database
    DATABASE_URL: str
    QDRANT_URL: str
    QDRANT_API_KEY: str

    # OpenAI
    OPENAI_API_KEY: str

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
```

## File Organization

### Frontend File Organization

```typescript
// frontend/src/components/ChatInterface/ChatInterface.tsx
import React, {useState} from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {chatWithRAG} from '@site/src/services/api';
import styles from './ChatInterface.module.css';

export default function ChatInterface() {
  const {siteConfig} = useDocusaurusContext();
  const apiUrl = siteConfig.customFields.apiUrl as string;

  // Component logic...
}
```

```typescript
// frontend/src/services/api.ts
const API_BASE_URL = window.API_CONFIG?.baseUrl || 'http://localhost:8000';

export async function chatWithRAG(query: string): Promise<ChatResponse> {
  const response = await fetch(`${API_BASE_URL}/api/v1/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({query}),
  });

  if (!response.ok) {
    throw new Error('Failed to chat with RAG');
  }

  return response.json();
}
```

### Backend File Organization

```python
# backend/app/api/routes/chat.py
from fastapi import APIRouter, Depends, HTTPException
from app.models.schemas import ChatRequest, ChatResponse
from app.services.rag import RAGService
from app.api.deps import get_rag_service

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    rag_service: RAGService = Depends(get_rag_service),
):
    """Handle chat requests with RAG pipeline"""
    try:
        response = await rag_service.generate_response(request.query)
        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

### Environment Files

```bash
# .env.example (copy to .env and fill in values)

# Backend API
API_URL=http://localhost:8000

# Database
DATABASE_URL=postgresql://user:password@localhost:5432/physical_ai
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI
OPENAI_API_KEY=your-openai-api-key

# Frontend
FRONTEND_URL=http://localhost:3000
```

## Development Workflow

### Local Development Setup

```bash
# 1. Clone repository
git clone https://github.com/your-org/physical-ai-book.git
cd physical-ai-book

# 2. Copy environment files
cp shared/config/.env.example .env

# 3. Install dependencies
make install

# 4. Start databases (Docker Compose)
docker-compose -f shared/docker/docker-compose.yml up -d

# 5. Initialize database
cd backend && python scripts/init_db.py

# 6. Ingest initial documents
cd backend && python scripts/ingest_documents.py

# 7. Start development servers
make dev  # Runs both frontend and backend
```

### Docker Compose for Development

```yaml
# shared/docker/docker-compose.yml
version: '3.8'

services:
  postgres:
    image: postgres:15
    environment:
      POSTGRES_USER: physical_ai_user
      POSTGRES_PASSWORD: dev_password
      POSTGRES_DB: physical_ai
    ports:
      - "5432:5432"
    volumes:
      - postgres_data:/var/lib/postgresql/data

  qdrant:
    image: qdrant/qdrant:latest
    ports:
      - "6333:6333"
    volumes:
      - qdrant_data:/qdrant/storage

volumes:
  postgres_data:
  qdrant_data:
```

## Deployment Considerations

### Vercel (Frontend) + Railway/Render (Backend)

**Frontend deployment (Vercel):**
```json
// vercel.json
{
  "buildCommand": "cd frontend && npm run build",
  "outputDirectory": "frontend/build",
  "env": {
    "API_URL": "@api-url-production"
  }
}
```

**Backend deployment (Railway):**
```toml
# railway.toml
[build]
builder = "NIXPACKS"
buildCommand = "pip install -r requirements.txt"

[deploy]
startCommand = "uvicorn app.main:app --host 0.0.0.0 --port $PORT"
healthcheckPath = "/api/v1/health"
healthcheckTimeout = 300
restartPolicyType = "ON_FAILURE"
```

### Monolithic Deployment (Single Server)

```dockerfile
# Dockerfile for combined deployment
FROM node:18 AS frontend-builder
WORKDIR /app/frontend
COPY frontend/package*.json ./
RUN npm install
COPY frontend/ ./
RUN npm run build

FROM python:3.11
WORKDIR /app

# Copy backend
COPY backend/ ./backend/
RUN pip install -r backend/requirements.txt

# Copy frontend build
COPY --from=frontend-builder /app/frontend/build ./frontend/build

# Install nginx to serve frontend
RUN apt-get update && apt-get install -y nginx

# Configure nginx
COPY nginx.conf /etc/nginx/nginx.conf

# Start script
COPY start.sh /app/
RUN chmod +x /app/start.sh

CMD ["/app/start.sh"]
```

### Environment Variable Management

```typescript
// frontend/src/config.ts
export const config = {
  apiUrl: process.env.REACT_APP_API_URL || window.API_CONFIG?.baseUrl || 'http://localhost:8000',
  environment: process.env.NODE_ENV,
};
```

```python
# backend/app/core/config.py
import os
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    # Automatically loads from environment variables
    API_V1_STR: str = "/api/v1"
    DATABASE_URL: str = os.getenv("DATABASE_URL", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")

    # CORS - read from comma-separated string
    ALLOWED_ORIGINS: list[str] = os.getenv(
        "ALLOWED_ORIGINS",
        "http://localhost:3000"
    ).split(",")

    class Config:
        env_file = ".env"
```

## Critical Integration Checklist

- [ ] Backend API URL configured in frontend (dev and prod)
- [ ] CORS origins configured in backend
- [ ] API client error handling in frontend
- [ ] Environment variables documented
- [ ] Docker Compose for local development
- [ ] Database migrations automated
- [ ] Health check endpoints implemented
- [ ] Logging configured in both layers
- [ ] Error tracking (Sentry, etc.)
- [ ] API documentation (Swagger/OpenAPI)
- [ ] Frontend-backend version compatibility
- [ ] Deployment scripts tested
