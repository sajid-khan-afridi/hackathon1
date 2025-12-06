# Quick Start Guide - Physical AI RAG Chatbot

**Goal**: Get the chatbot running locally in under 5 minutes.

---

## Prerequisites

- ✅ Node.js 18+ installed
- ✅ Python 3.11+ installed
- ✅ PostgreSQL database (Neon or local)
- ✅ Qdrant instance (cloud or Docker)
- ✅ OpenAI API key

---

## Step 1: Clone and Setup (2 minutes)

```bash
# Navigate to project
cd "D:\GitHub Connected\hackathon1"

# Install frontend dependencies
npm install

# Install backend dependencies
cd backend
pip install -r requirements.txt
cd ..
```

---

## Step 2: Configure Environment (1 minute)

Create/update `backend/.env`:

```bash
# Database (Neon Postgres)
DATABASE_URL=postgresql://user:password@host:5432/physical_ai

# Qdrant Vector DB
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI
OPENAI_API_KEY=your-openai-api-key

# CORS (for local dev)
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000
```

**Quick Setup with Docker** (if you don't have Postgres/Qdrant):
```bash
# In shared/docker directory
docker-compose up -d
```

---

## Step 3: Start Backend (30 seconds)

```bash
cd backend
uvicorn app.main:app --reload --port 8000
```

**Expected Output**:
```
INFO:     Uvicorn running on http://127.0.0.1:8000
INFO:     Starting Physical AI RAG API...
```

**Test Health**:
```bash
curl http://localhost:8000/api/v1/health
```

**Expected**:
```json
{
  "status": "healthy",
  "services": {
    "database": "healthy",
    "qdrant": "healthy",
    "openai": "healthy"
  },
  "version": "0.1.0"
}
```

---

## Step 4: Start Frontend (30 seconds)

Open a new terminal:

```bash
npm start
```

**Expected**:
- Opens http://localhost:3000 automatically
- Docusaurus dev server running
- Chat widget visible (floating button bottom-right)

---

## Step 5: Test Integration (1 minute)

### Manual Test

1. **Open browser**: http://localhost:3000
2. **Click floating chat button** (bottom-right, purple)
3. **Type message**: "What is ROS2?"
4. **Press Enter**
5. **Verify**:
   - Loading dots appear
   - Response displays
   - Sources shown with citations
   - Citations are clickable

### Automated Test

```bash
python backend/test_integration.py
```

**Expected**: All 4 tests pass ✅

---

## Common Issues

### Issue: Backend fails to start

**Error**: `ModuleNotFoundError: No module named 'fastapi'`

**Fix**:
```bash
cd backend
pip install -r requirements.txt
```

---

### Issue: Database connection error

**Error**: `could not connect to server`

**Fix**:
1. Verify `DATABASE_URL` in `.env`
2. Check database is running
3. Test connection:
   ```bash
   psql $DATABASE_URL
   ```

---

### Issue: CORS error in browser

**Error**: `Access-Control-Allow-Origin header missing`

**Fix**:
1. Verify backend is running on port 8000
2. Check `ALLOWED_ORIGINS` in backend/.env includes `http://localhost:3000`
3. Restart backend

---

### Issue: Chat widget not visible

**Fix**:
1. Open browser DevTools Console
2. Check for errors
3. Verify `static/js/api-config.js` loaded (Network tab)
4. Clear browser cache and refresh

---

## Development Workflow

### Backend Development

```bash
cd backend

# Run with auto-reload
uvicorn app.main:app --reload --port 8000

# View logs
# Logs appear in terminal

# API documentation (interactive)
# Open: http://localhost:8000/api/v1/docs
```

### Frontend Development

```bash
# Run dev server
npm start

# Build for production
npm run build

# Type checking
npx tsc --noEmit

# Linting
npm run lint
```

---

## API Endpoints

### Health Check
```
GET /api/v1/health
```

### Chat
```
POST /api/v1/chat
Body: {
  "query": "What is ROS2?",
  "conversation_id": "optional-uuid",
  "top_k": 8
}
```

### Documents
```
GET /api/v1/documents?module=module-01-ros2&chapter=01-fundamentals
```

### Search (Debug)
```
POST /api/v1/search
Body: {
  "query": "ROS2 publisher",
  "top_k": 5
}
```

---

## Project Structure

```
D:\GitHub Connected\hackathon1\
├── backend/                      # FastAPI backend
│   ├── app/
│   │   ├── api/                  # API routes
│   │   ├── core/                 # Config, security
│   │   ├── models/               # Schemas, database
│   │   └── services/             # RAG, embeddings, storage
│   └── test_integration.py       # Integration tests
│
├── src/                          # Docusaurus frontend
│   ├── components/
│   │   └── ChatInterface/        # Chat widget
│   ├── services/
│   │   └── api.ts                # API client
│   ├── types/
│   │   └── api.ts                # TypeScript types
│   └── theme/
│       └── Root.tsx              # Global wrapper
│
├── static/
│   └── js/
│       └── api-config.js         # API base URL
│
└── docs/                         # Physical AI book content
```

---

## Next Steps

### 1. Ingest Content

```bash
cd backend
python scripts/ingest_documents.py
```

### 2. Test Queries

Try these example queries in the chat:
- "What is ROS2?"
- "How do I create a publisher in ROS2?"
- "Explain Gazebo simulation"
- "What is a vision-language-action model?"

### 3. Monitor Performance

```bash
# Backend logs show:
# - Query processing time
# - Retrieval time
# - Generation time
# - Error details
```

---

## Helpful Commands

```bash
# Restart everything
npm start                          # Frontend (Terminal 1)
uvicorn app.main:app --reload      # Backend (Terminal 2)

# Check TypeScript
npx tsc --noEmit

# Run integration tests
python backend/test_integration.py

# View API docs
# Open: http://localhost:8000/api/v1/docs

# View frontend
# Open: http://localhost:3000
```

---

## Support

If you encounter issues:

1. Check `TROUBLESHOOTING.md`
2. Review backend logs (terminal output)
3. Check browser console (F12 → Console)
4. Verify environment variables in `.env`

---

**Ready to go!** 🚀

Your chatbot should now be running at http://localhost:3000 with a functional RAG pipeline.
