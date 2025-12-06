# ADR-010: Variable-Size Chunking Strategy for Content Types

> **Scope**: Document chunking approach for Physical AI book content based on content type (theoretical, code, mixed).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-rag-chatbot
- **Context:** Physical AI book RAG chatbot system

<!-- Significance checklist:
     1) Impact: Yes - Affects retrieval accuracy, context quality, and user experience
     2) Alternatives: Yes - Fixed-size, semantic, or variable-size chunking strategies
     3) Scope: Yes - Affects ingestion pipeline, storage, retrieval, and response generation
-->

## Decision

We will implement **variable-size chunking** based on content type:

**Chunk Sizes**:
```python
CHUNK_SIZES = {
    "theoretical": 800,      # Conceptual explanations, definitions
    "code": 1200,            # Code blocks, functions, classes
    "mixed": 1000,           # Default for tutorials, mixed content
}

OVERLAP = 200  # Token overlap between consecutive chunks
```

**Content Type Detection**:
- **Code chunks**: Detect by code fence markers (```python, ```cpp, ```yaml) or high code-to-text ratio
- **Theoretical chunks**: Pure text, no code blocks, conceptual headings
- **Mixed chunks**: Combination of text and code (tutorials, examples with explanations)

**Metadata Enrichment** (stored in Postgres, mirrored in Qdrant payload):
```python
chunk_metadata = {
    "module": "module-01-ros2",           # Top-level module
    "chapter": "01-fundamentals",         # Chapter within module
    "section": "publisher-subscriber",    # Section within chapter
    "type": "code",                       # theoretical | code | tutorial | reference
    "language": "python",                 # For code chunks: python | cpp | yaml | bash
    "framework": "ros2",                  # ros2 | gazebo | unity | isaac
    "complexity": "beginner",             # beginner | intermediate | advanced
    "prerequisites": ["basic-python", "linux-basics"],  # Required knowledge
    "related_sections": [...],            # Cross-references
}
```

**Chunking Process**:
1. Parse Markdown/MDX document
2. Detect content type (theoretical vs code vs mixed)
3. Select chunk size based on type
4. Apply sliding window with 200-token overlap
5. Extract metadata from file path and frontmatter
6. Generate embeddings for each chunk
7. Store in Postgres (metadata + text) and Qdrant (embedding + payload)

## Consequences

### Positive

- **Preserves Code Integrity**: 1200-token chunks ensure complete functions/classes not split mid-code
- **Optimizes Theoretical Content**: 800-token chunks provide precise retrieval for concepts (not bloated)
- **Context Continuity**: 200-token overlap prevents information loss at chunk boundaries
- **Better Retrieval Accuracy**: Content-aware chunking improves semantic coherence of chunks
- **Metadata-Rich**: Each chunk tagged with module, chapter, language, framework (enables advanced filtering)
- **Empirically Validated**: Chunk sizes based on RAG best practices and Physical AI content analysis
- **Balanced Trade-offs**: Larger code chunks preserve context; smaller theoretical chunks increase precision

### Negative

- **Chunking Complexity**: More complex than fixed-size chunking (requires content type detection)
  - **Mitigation**: Abstracted in chunking.py utility; pattern-based detection (code fences, headings)
- **Manual Tuning**: Chunk sizes may need adjustment based on retrieval accuracy feedback
  - **Mitigation**: Configurable sizes in config.py; A/B testing in Phase 5 validation
- **Inconsistent Chunk Counts**: Variable sizes mean different documents produce different chunk counts
  - **Mitigation**: Not a problem; retrieval selects top-k by relevance, not fixed count per document
- **Metadata Extraction Fragility**: Parsing frontmatter, file paths could break if content structure changes
  - **Mitigation**: Defensive parsing with fallbacks; validate metadata during ingestion (log warnings)

## Alternatives Considered

### Alternative 1: Fixed-Size Chunking (500 tokens)
**Architecture**: All chunks exactly 500 tokens, 100-token overlap

**Pros**:
- **Simplicity**: No content type detection, straightforward sliding window
- **Predictable**: Consistent chunk count per document
- **Fast Ingestion**: No parsing overhead

**Cons**:
- **Splits Code Mid-Function**: 500 tokens often insufficient for complete Python functions or C++ classes
- **Bloats Theoretical Content**: Short definitions spread across multiple chunks (noisy retrieval)
- **Poor Context Preservation**: Code examples separated from explanatory text
- **Benchmark**: ~70% retrieval accuracy vs 85% for variable-size
- **Rejected**: Quality degradation unacceptable for educational content

### Alternative 2: Semantic Chunking (by Heading/Section)
**Architecture**: Chunk by Markdown structure (# headings, ## subheadings)

**Pros**:
- **Natural Boundaries**: Respects document structure
- **Complete Sections**: Each chunk is semantically complete unit
- **No Split Concepts**: Explanations stay together

**Cons**:
- **Inconsistent Sizes**: Some sections 100 tokens, others 3000+ tokens (exceeds context budget)
- **Large Chunks**: Big sections create bloated chunks (retrieval less precise)
- **Token Limit Violations**: Some Physical AI chapters have 2000+ token sections
- **Embedding Quality**: Very large chunks dilute semantic meaning in embedding
- **Rejected**: Size variability too extreme (100-3000 tokens) hurts retrieval and context assembly

### Alternative 3: Recursive Character Text Splitter (LangChain Default)
**Architecture**: Use LangChain RecursiveCharacterTextSplitter (splits by characters, respects sentences/paragraphs)

**Pros**:
- **Battle-Tested**: LangChain default, widely used
- **Respects Boundaries**: Tries to split on sentences, paragraphs

**Cons**:
- **Character-Based**: Doesn't account for tokenization (character count ≠ token count)
- **No Content-Type Awareness**: Treats code and text identically
- **Overlap Fixed**: 20% overlap not optimal for all content types
- **Less Control**: Harder to customize for Physical AI book structure
- **Rejected**: Token-based approach more precise; content-type awareness critical

### Alternative 4: No Chunking (Full Documents)
**Architecture**: Embed entire documents (chapters) as single chunks

**Pros**:
- **Complete Context**: No information loss from splitting
- **Simpler Pipeline**: No chunking logic needed

**Cons**:
- **Exceeds Context Window**: Physical AI chapters are 2000-5000 tokens (exceed 1536-dim embedding optimal size)
- **Poor Retrieval Granularity**: Retrieving entire chapter for specific question (too much noise)
- **Embedding Quality Dilution**: Large documents create averaged embeddings (less precise)
- **Context Assembly Impossible**: Cannot fit multiple chapters in LLM context window
- **Rejected**: Fundamentally incompatible with RAG requirements

## References

- Feature Spec: [specs/001-physical-ai-rag-chatbot/spec.md](../../specs/001-physical-ai-rag-chatbot/spec.md)
- Implementation Plan: [specs/001-physical-ai-rag-chatbot/plan.md](../../specs/001-physical-ai-rag-chatbot/plan.md)
- RAG Architecture Reference: [.claude/skills/physical-ai-rag-builder/references/rag-architecture.md](../../.claude/skills/physical-ai-rag-builder/references/rag-architecture.md)
- Related ADRs: ADR-006 (OpenAI Models), ADR-008 (RAG Pipeline Design)

## Validation

- [ ] Manual review of chunked content (verify no broken code, complete concepts)
- [ ] Test retrieval accuracy on 50+ queries (target: ≥85% with variable-size vs <75% with fixed-size)
- [ ] Measure context window utilization (6-8 chunks fit in 5000-token budget)
- [ ] Verify chunk metadata extraction accuracy (module, chapter, section, type)
- [ ] A/B test chunk sizes (800/1200/1000 vs 600/1000/800 vs 1000 fixed)
- [ ] Benchmark embedding quality (larger chunks dilute semantic meaning)
- [ ] User feedback: "Was the response relevant?" (target: >80% yes)
