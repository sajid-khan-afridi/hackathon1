# Physical AI RAG Backend

FastAPI backend for the Physical AI RAG chatbot system.

## Features

- **RAG Pipeline**: Retrieval-Augmented Generation for book content queries
- **Hybrid Storage**: PostgreSQL (metadata) + Qdrant (vector embeddings)
- **OpenAI Integration**: GPT-4 for generation, text-embedding-3-small for embeddings
- **Hybrid Search**: 70% vector similarity + 30% BM25 keyword matching
- **RESTful API**: FastAPI with OpenAPI documentation

## Quick Start

### Prerequisites

- Python 3.11+
- PostgreSQL 15+ (or use Docker Compose)
- Qdrant (or use Docker Compose)
- OpenAI API key

### Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Copy environment variables:
```bash
cp ../shared/config/.env.example ../.env
```

3. Edit `.env` and add your OpenAI API key

4. Start databases (Docker):
```bash
docker-compose -f ../shared/docker/docker-compose.yml up -d postgres qdrant
```

5. Initialize database:
```bash
python scripts/init_db.py
```

6. Ingest documents:
```bash
python scripts/ingest_documents.py
```

7. Start development server:
```bash
uvicorn app.main:app --reload --port 8000
```

## API Endpoints

### Chat
- **POST /api/v1/chat**: Submit a question and get RAG-generated response
  ```json
  {
    "query": "What is ROS2?"
  }
  ```

### Health
- **GET /api/v1/health**: Check system health and service status

### Documents
- **GET /api/v1/documents**: List documents with optional filtering
  - Query params: `module`, `chapter`

### Search
- **POST /api/v1/search**: Test retrieval without generation (debugging)

## Project Structure

```
backend/
├── app/
│   ├── api/              # API routes
│   │   ├── routes/       # Endpoint handlers
│   │   └── deps.py       # Dependency injection
│   ├── core/             # Core configuration
│   │   ├── config.py     # Settings
│   │   └── security.py   # Auth utilities
│   ├── models/           # Data models
│   │   ├── database.py   # SQLAlchemy models
│   │   └── schemas.py    # Pydantic schemas
│   ├── services/         # Business logic
│   │   ├── rag.py        # RAG pipeline
│   │   ├── embeddings.py # OpenAI embeddings
│   │   ├── retrieval.py  # Hybrid search
│   │   └── storage.py    # Database operations
│   ├── utils/            # Utilities
│   │   ├── chunking.py   # Document chunking
│   │   └── logging.py    # Structured logging
│   └── main.py           # FastAPI app
├── migrations/           # Alembic migrations
├── scripts/              # Utility scripts
├── tests/                # Test suite
└── requirements.txt
```

## Development

### Run Tests
```bash
pytest tests/ -v --cov=app
```

### Format Code
```bash
black app/ tests/
```

### Lint Code
```bash
flake8 app/ tests/
mypy app/
```

### Create Migration
```bash
alembic revision --autogenerate -m "Description"
alembic upgrade head
```

## Environment Variables

See `../shared/config/.env.example` for all available configuration options.

Key variables:
- `DATABASE_URL`: PostgreSQL connection string
- `QDRANT_URL`: Qdrant server URL
- `OPENAI_API_KEY`: OpenAI API key
- `ALLOWED_ORIGINS`: CORS allowed origins

## Documentation

- API docs: http://localhost:8000/api/v1/docs
- ReDoc: http://localhost:8000/api/v1/redoc
- Architecture: See `../../specs/001-physical-ai-rag-chatbot/plan.md`
