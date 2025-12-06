# ADR-006: OpenAI Model Selection for RAG Pipeline

> **Scope**: Selection of OpenAI models for both embedding generation and response generation in the RAG chatbot system.

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-rag-chatbot
- **Context:** Physical AI book RAG chatbot system

<!-- Significance checklist:
     1) Impact: Yes - Core component of RAG pipeline affecting quality, cost, and performance
     2) Alternatives: Yes - Multiple OpenAI embedding and generation models with different tradeoffs
     3) Scope: Yes - Affects retrieval accuracy, response quality, storage costs, API costs
-->

## Decision

We will use the following OpenAI model combination for the RAG pipeline:

**Embedding Model: text-embedding-3-small (1536 dimensions)**
- Vector dimension: 1536 (optimal for Qdrant storage efficiency)
- Cost: $0.00002 per 1K tokens
- Quality: Sufficient for technical content retrieval
- Performance: Fast embedding generation

**Generation Model: GPT-4-turbo (128K context window)**
- Context window: 128K tokens (ample for retrieved chunks + conversation history)
- Quality: Superior reasoning for educational content and technical explanations
- Cost: Higher per token, but justified by quality requirements
- Capabilities: Code understanding, citation generation, technical accuracy

**Caching Strategy:**
- Embedding cache: In-memory LRU cache (80% cache hit rate target)
- Target: Reduce redundant embedding API calls for frequently queried content
- Query cache: Disabled for v1 (educational context requires fresh responses)

## Consequences

### Positive

- **High-Quality Embeddings**: text-embedding-3-small provides excellent semantic understanding for technical content at reasonable cost
- **Storage Efficiency**: 1536 dimensions balance quality and storage requirements (~60KB per chunk in Qdrant)
- **Superior Response Quality**: GPT-4-turbo delivers accurate, well-reasoned responses for educational content with proper citation support
- **Large Context Window**: 128K context supports comprehensive retrieval (6-8 chunks) plus conversation history
- **Cost-Effective Balance**: Embedding model minimizes storage and API costs while generation model prioritizes quality
- **Code Understanding**: GPT-4-turbo excels at explaining ROS2, Gazebo, Unity code examples from book content
- **Proven Stack**: OpenAI models are well-documented, reliable, and widely adopted for RAG systems

### Negative

- **Higher Generation Cost**: GPT-4-turbo is more expensive than GPT-3.5-turbo (~10x per token)
  - **Mitigation**: Acceptable for educational platform with good-faith usage; implement monitoring to detect anomalies
- **API Dependency**: Single vendor (OpenAI) creates dependency risk
  - **Mitigation**: Abstraction layer in embeddings.py service allows model swapping if needed
- **Storage Requirements**: 1536-dim embeddings require ~60KB per chunk (~600MB for 10K chunks)
  - **Mitigation**: Within Qdrant Cloud free tier (1GB limit); can upgrade if needed
- **Rate Limiting Risk**: OpenAI API rate limits could throttle performance
  - **Mitigation**: Implement exponential backoff, queue management, and usage monitoring

## Alternatives Considered

### Alternative 1: text-embedding-ada-002 + GPT-3.5-turbo
**Embedding**: text-embedding-ada-002 (1536 dimensions, $0.0001/1K tokens)
**Generation**: GPT-3.5-turbo (16K context, $0.0005/1K input tokens)

**Pros**:
- Lower total cost (5x cheaper for embeddings, 10x cheaper for generation)
- Adequate quality for simple Q&A
- Faster response times

**Cons**:
- **text-embedding-ada-002**: Older embedding model with lower quality than text-embedding-3-small
- **GPT-3.5-turbo**: Inferior reasoning for complex technical explanations
- **Quality Gap**: Educational content requires accurate, nuanced responses; GPT-3.5 struggles with multi-step technical reasoning
- **Rejected**: Cost savings not worth quality degradation for educational platform

### Alternative 2: text-embedding-3-large + GPT-4-turbo
**Embedding**: text-embedding-3-large (3072 dimensions, $0.00013/1K tokens)
**Generation**: GPT-4-turbo (same)

**Pros**:
- Highest quality embeddings (best retrieval accuracy)
- Same superior generation quality

**Cons**:
- **2x Storage**: 3072 dimensions require ~120KB per chunk (~1.2GB for 10K chunks, exceeds Qdrant free tier)
- **6.5x Embedding Cost**: $0.00013 vs $0.00002 per 1K tokens
- **Diminishing Returns**: text-embedding-3-small already provides excellent quality for technical content
- **Rejected**: Storage and cost increase not justified by marginal quality improvement

### Alternative 3: OpenAI Embeddings + Anthropic Claude 3
**Embedding**: text-embedding-3-small (1536 dimensions)
**Generation**: Claude 3 Opus (200K context)

**Pros**:
- Larger context window (200K vs 128K)
- Competitive quality with GPT-4
- Vendor diversification

**Cons**:
- **Integration Complexity**: Mixing OpenAI and Anthropic SDKs increases code complexity
- **Cost**: Claude 3 Opus comparable cost to GPT-4-turbo
- **Ecosystem**: GPT-4 has more RAG-specific examples and tooling
- **Rejected**: Added complexity not justified; 128K context window sufficient

## References

- Feature Spec: [specs/001-physical-ai-rag-chatbot/spec.md](../../specs/001-physical-ai-rag-chatbot/spec.md)
- Implementation Plan: [specs/001-physical-ai-rag-chatbot/plan.md](../../specs/001-physical-ai-rag-chatbot/plan.md)
- RAG Architecture Reference: [.claude/skills/physical-ai-rag-builder/references/rag-architecture.md](../../.claude/skills/physical-ai-rag-builder/references/rag-architecture.md)
- Related ADRs: ADR-007 (Hybrid Storage), ADR-008 (RAG Pipeline Design)

## Validation

- [ ] Compare retrieval accuracy across embedding models (text-embedding-3-small vs ada-002 vs 3-large)
- [ ] A/B test response quality between GPT-4-turbo and GPT-3.5-turbo on sample educational queries
- [ ] Monitor API costs during Phase 3 implementation (target: <$10/month for development)
- [ ] Measure embedding cache hit rate (target: >80%)
- [ ] Benchmark response quality on 20+ test queries covering conceptual, code, and troubleshooting types
