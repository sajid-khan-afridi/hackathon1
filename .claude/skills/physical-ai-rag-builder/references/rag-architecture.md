# RAG Architecture Patterns for Physical AI Book

## Table of Contents
- [Core RAG Pipeline](#core-rag-pipeline)
- [Document Processing](#document-processing)
- [Retrieval Strategies](#retrieval-strategies)
- [Context Assembly](#context-assembly)
- [Response Generation](#response-generation)
- [Physical AI Specific Considerations](#physical-ai-specific-considerations)

## Core RAG Pipeline

### Standard Flow
1. **Ingestion**: Document → Chunks → Embeddings → Vector Store
2. **Query**: User Question → Query Embedding
3. **Retrieval**: Hybrid Search (Vector + Keyword)
4. **Assembly**: Retrieved Chunks → Context Window
5. **Generation**: LLM (Context + Query) → Response

### Physical AI Book Specialization
- **Technical depth**: Mix of theoretical concepts and practical code
- **Multi-modal content**: Text, diagrams, code snippets, mathematical formulas
- **Hierarchical structure**: Modules → Chapters → Sections
- **Code-heavy**: ROS2, Gazebo, Unity integration examples

## Document Processing

### Chunking Strategy
```python
# Recommended chunk sizes for Physical AI content
CHUNK_SIZES = {
    "theoretical": 800,      # Concepts, explanations
    "code": 1200,            # Code blocks with context
    "mixed": 1000,           # Default for mixed content
}

OVERLAP = 200  # Token overlap between chunks
```

### Chunk Types
1. **Conceptual chunks**: Definitions, theory, explanations
2. **Code chunks**: Complete functions, classes with docstrings
3. **Tutorial chunks**: Step-by-step instructions with code
4. **Reference chunks**: API docs, configuration examples

### Metadata Enrichment
```python
chunk_metadata = {
    "module": "module-01-ros2",
    "chapter": "01-fundamentals",
    "section": "publisher-subscriber",
    "type": "code",  # theoretical | code | tutorial | reference
    "language": "python",  # for code chunks
    "framework": "ros2",
    "complexity": "beginner",  # beginner | intermediate | advanced
    "prerequisites": ["basic-python", "linux-basics"],
    "related_sections": [...],
}
```

## Retrieval Strategies

### Hybrid Search (Vector + BM25)
```python
# Optimal weighting for technical content
WEIGHTS = {
    "vector_score": 0.7,
    "bm25_score": 0.3,
}

# Adjust based on query type
def adjust_weights(query_type):
    if query_type == "conceptual":
        return {"vector": 0.8, "bm25": 0.2}
    elif query_type == "code_search":
        return {"vector": 0.5, "bm25": 0.5}
    elif query_type == "troubleshooting":
        return {"vector": 0.6, "bm25": 0.4}
```

### Query Classification
```python
query_patterns = {
    "conceptual": ["what is", "explain", "define", "how does"],
    "code": ["code for", "implement", "example of", "how to"],
    "troubleshooting": ["error", "fix", "not working", "issue with"],
    "comparison": ["difference between", "vs", "compare"],
}
```

### Reranking
1. **Semantic reranking**: Use cross-encoder for top 20 results
2. **Metadata filtering**: Prefer chunks matching user's learning level
3. **Recency**: Prefer newer content for framework-specific queries
4. **Code completeness**: Prioritize chunks with complete code examples

## Context Assembly

### Context Window Strategy (for GPT-4)
```
Total: 8k tokens
├── System Prompt: 500 tokens
├── Retrieved Context: 5000 tokens (6-8 chunks)
├── Conversation History: 1500 tokens
└── Response Buffer: 1000 tokens
```

### Context Ordering
1. **Most relevant chunk** (highest combined score)
2. **Code example** (if available and relevant)
3. **Prerequisites** (if user is beginner)
4. **Related concepts** (for comprehensive answer)

### Deduplication
- Remove duplicate chunks (by content hash)
- Merge overlapping chunks from same section
- Consolidate code examples from same file

## Response Generation

### Prompt Template for Physical AI
```python
system_prompt = """You are an expert assistant for the Physical AI book.
The book teaches robotics, simulation, and AI integration using ROS2, Gazebo, Unity, and NVIDIA Isaac.

When answering:
1. Cite specific sections (Module X, Chapter Y)
2. Provide code examples when relevant
3. Explain prerequisites for advanced topics
4. Link to related concepts
5. Warn about common pitfalls

Context from the book:
{retrieved_context}

User question: {query}
"""
```

### Citation Strategy
```python
# Add source attribution to every response
citation_format = """
**Sources:**
- Module 01, Chapter 02: ROS2 Fundamentals (pg. 45-52)
- Module 03, Section 3.1: Gazebo Integration (pg. 112-115)
"""
```

## Physical AI Specific Considerations

### Framework Version Management
- Track ROS2 version (Humble, Iron, etc.)
- Track Gazebo version (Classic vs. Sim)
- Track Unity version compatibility
- Track NVIDIA Isaac SDK version

### Code Execution Context
- Provide environment setup instructions
- Include dependency installation
- Mention OS requirements (Ubuntu 22.04, etc.)
- Reference Docker configurations if available

### Multi-Framework Integration
```python
# Tag chunks with framework dependencies
frameworks = {
    "ros2": ["foxy", "humble", "iron"],
    "gazebo": ["classic-11", "sim-7", "sim-8"],
    "unity": ["2021.3", "2022.3"],
    "isaac": ["2023.1"],
}
```

### Learning Path Awareness
```python
# Recommend prerequisite sections
learning_paths = {
    "beginner": ["module-01-ros2", "module-02-gazebo-basics"],
    "intermediate": ["module-02-gazebo-unity", "module-03-isaac-intro"],
    "advanced": ["module-03-isaac-advanced", "module-04-vla"],
}
```
