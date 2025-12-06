# RAG Pipeline Quick Start Guide

## Prerequisites

1. **Environment Setup**:
   ```bash
   # Ensure .env file has required keys
   OPENAI_API_KEY=your_openai_api_key_here
   DATABASE_URL=postgresql://...
   QDRANT_URL=http://localhost:6333
   ```

2. **Dependencies**:
   ```bash
   pip install openai sqlalchemy qdrant-client asyncpg
   ```

3. **Database Schema**:
   - Ensure `documents` and `chunks` tables exist
   - Ensure `user_queries` and `chunk_usage` tables exist
   - Run migrations if needed

---

## Running Tests

### Option 1: Full Test Suite
```bash
cd backend
python test_rag_pipeline.py
```

### Option 2: Individual Tests
```python
import asyncio
from app.services.rag import test_embeddings, test_retrieval, test_rag_pipeline

# Test embeddings only
asyncio.run(test_embeddings())

# Test retrieval only
asyncio.run(test_retrieval())

# Test full RAG pipeline
asyncio.run(test_rag_pipeline())
```

---

## Using the RAG Service

### Basic Usage

```python
from app.services.rag import rag_service

# Simple query
result = await rag_service.query(
    query_text="What is ROS2?",
    user_id="user123"
)

print(result["response"])
print(f"Query type: {result['query_type']}")
print(f"Time: {result['total_time_ms']}ms")
print(f"Sources: {len(result['sources'])} citations")
```

### Advanced Usage with Filters

```python
from app.services.retrieval import retrieval_service

# Retrieve with metadata filters
chunks = await retrieval_service.hybrid_search(
    query="How to create a ROS2 publisher?",
    top_k=10,
    filters={
        "module": "module-01-ros2",
        "chunk_type": "code",
        "learning_level": "beginner"
    }
)
```

### Query Types

The system automatically classifies queries into 4 types:

1. **Conceptual**: "What is X?", "Explain Y", "Define Z"
   - Weight: 80% vector, 20% BM25

2. **Code**: "Show me code for X", "How to implement Y"
   - Weight: 50% vector, 50% BM25

3. **Troubleshooting**: "Fix error X", "Debug Y"
   - Weight: 60% vector, 40% BM25

4. **Comparison**: "Difference between X and Y", "X vs Y"
   - Weight: 70% vector, 30% BM25

---

## Response Format

```python
{
    "response": "ROS2 (Robot Operating System 2) is...",
    "sources": [
        {
            "module": "module-01-ros2",
            "chapter": "01-fundamentals",
            "section": "Introduction",
            "url_fragment": "#ros2-fundamentals-introduction"
        }
    ],
    "query_type": "conceptual",
    "retrieval_time_ms": 245,
    "generation_time_ms": 1823,
    "total_time_ms": 2068,
    "chunks_used": 8
}
```

---

## Embeddings Service

### Generate Single Embedding

```python
from app.services.embeddings import embeddings_service

embedding = await embeddings_service.generate_embedding("Sample text")
print(f"Dimension: {len(embedding)}")  # 1536
```

### Batch Embedding Generation

```python
texts = ["Text 1", "Text 2", "Text 3"]
embeddings = await embeddings_service.batch_generate_embeddings(texts)
print(f"Generated {len(embeddings)} embeddings")
```

### Cache Statistics

```python
stats = embeddings_service.get_cache_stats()
print(f"Cache hit rate: {stats['hit_rate']:.2%}")
print(f"Cache size: {stats['cache_size']}")
```

---

## Retrieval Service

### Classify Query

```python
from app.services.retrieval import retrieval_service

query_type = await retrieval_service.classify_query("What is ROS2?")
print(f"Query type: {query_type}")  # "conceptual"
```

### Get Adaptive Weights

```python
weights = retrieval_service.get_search_weights("code")
print(weights)  # {"vector": 0.5, "bm25": 0.5}
```

### Hybrid Search

```python
results = await retrieval_service.hybrid_search(
    query="ROS2 publisher example",
    top_k=5,
    query_type="code"
)

for result in results:
    print(f"Score: {result['hybrid_score']:.4f}")
    print(f"Module: {result['module']}")
    print(f"Content: {result['content'][:100]}...")
```

---

## Performance Monitoring

### Check Response Time

```python
result = await rag_service.query("What is ROS2?")

if result['total_time_ms'] > 3000:
    print("⚠️ Slow response!")
else:
    print("✓ Good performance")
```

### Monitor Cache Effectiveness

```python
# Run multiple queries
for _ in range(10):
    await rag_service.query("What is ROS2?")

stats = embeddings_service.get_cache_stats()
print(f"Cache hit rate: {stats['hit_rate']:.2%}")
# Target: >50% for repeated queries
```

---

## Error Handling

### Handle API Errors

```python
try:
    result = await rag_service.query("What is ROS2?")
except ValueError as e:
    print(f"Invalid query: {e}")
except Exception as e:
    print(f"Error: {e}")
```

### Check for Empty Context

```python
result = await rag_service.query("Some obscure query")

if result['chunks_used'] == 0:
    print("⚠️ No relevant context found")
else:
    print(f"✓ Used {result['chunks_used']} chunks")
```

---

## Configuration

### Adjust Retrieval Parameters

Edit `backend/app/core/config.py`:

```python
# Number of chunks to retrieve
RETRIEVAL_TOP_K: int = 20  # Default: 20

# Context size limit
MAX_CONTEXT_TOKENS: int = 5000  # Default: 5000

# Response length limit
MAX_RESPONSE_TOKENS: int = 1000  # Default: 1000
```

### Adjust Cache Settings

```python
# Cache TTL (in seconds)
EMBEDDING_CACHE_TTL: int = 3600  # 1 hour

# Max cache size (hardcoded in embeddings.py)
# Change line 101: if len(self._cache) >= 1000:
```

---

## Common Issues & Solutions

### 1. OpenAI API Key Error

**Error**: `OpenAI API key not found`

**Solution**:
```bash
# Add to .env file
OPENAI_API_KEY=sk-...
```

### 2. Database Connection Error

**Error**: `Connection to database failed`

**Solution**:
```bash
# Check DATABASE_URL in .env
DATABASE_URL=postgresql://user:pass@host:5432/db
```

### 3. Qdrant Connection Error

**Error**: `Qdrant connection failed`

**Solution**:
```bash
# Ensure Qdrant is running
docker run -p 6333:6333 qdrant/qdrant

# Or update QDRANT_URL in .env
QDRANT_URL=http://localhost:6333
```

### 4. No Results Returned

**Issue**: Search returns empty results

**Solution**:
- Ensure documents are ingested into database
- Check that Qdrant collection exists
- Verify chunk embeddings are stored

### 5. Slow Response Time

**Issue**: Response takes >3s

**Solutions**:
- Check OpenAI API latency
- Optimize database queries
- Reduce `RETRIEVAL_TOP_K` value
- Enable caching for repeated queries

---

## Integration with FastAPI

### Example Endpoint

```python
from fastapi import APIRouter
from app.services.rag import rag_service
from app.models.schemas import ChatRequest, ChatResponse

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """Chat endpoint using RAG pipeline"""

    result = await rag_service.query(
        query_text=request.query,
        user_id=request.user_id
    )

    return ChatResponse(
        response=result["response"],
        sources=result["sources"]
    )
```

---

## Best Practices

1. **Always validate input**:
   ```python
   if not query or len(query) > 500:
       raise ValueError("Invalid query")
   ```

2. **Use caching for repeated queries**:
   - Embeddings are automatically cached
   - Consider adding response caching for common queries

3. **Monitor performance**:
   - Log all timing metrics
   - Alert on p95 latency >2s

4. **Handle errors gracefully**:
   - Always catch OpenAI API errors
   - Provide fallback responses

5. **Track analytics**:
   - All queries are logged to `user_queries` table
   - Use chunk usage data to improve retrieval

---

## Next Steps

1. **Run tests** to validate implementation
2. **Integrate with API endpoints** in FastAPI
3. **Connect to frontend** chat interface
4. **Monitor performance** in production
5. **Iterate based on user feedback**

---

For detailed implementation information, see `RAG_IMPLEMENTATION_SUMMARY.md`.
