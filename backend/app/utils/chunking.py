"""Document chunking utilities for RAG pipeline"""

import re
from typing import List, Dict, Any, Tuple
import tiktoken
from app.core.config import settings


def count_tokens(text: str, model: str = "gpt-3.5-turbo") -> int:
    """
    Count tokens using tiktoken

    Args:
        text: Text to count tokens for
        model: Model to use for tokenization

    Returns:
        Number of tokens
    """
    try:
        encoding = tiktoken.encoding_for_model(model)
        return len(encoding.encode(text))
    except Exception:
        # Fallback to rough estimate
        return len(text) // 4


def chunk_document(
    content: str,
    content_type: str,
    metadata: Dict[str, Any]
) -> List[Dict[str, Any]]:
    """
    Chunk document content based on type using variable-size strategy

    Args:
        content: Full document content
        content_type: Type of content ('theoretical', 'code', 'tutorial', 'reference')
        metadata: Document metadata (module, chapter, section)

    Returns:
        List of chunks with content and metadata

    Chunking strategy:
        - Theoretical content: 800 tokens (dense concepts)
        - Code blocks: 1200 tokens (preserve complete functions)
        - Mixed content: 1000 tokens (balance)
        - 200 token overlap between chunks
        - Smart splitting at paragraph/heading/code block boundaries
    """
    # Determine chunk size based on content type
    if content_type == "theoretical":
        chunk_size = settings.CHUNK_SIZE_THEORETICAL
    elif content_type == "code":
        chunk_size = settings.CHUNK_SIZE_CODE
    else:
        chunk_size = settings.CHUNK_SIZE_MIXED

    overlap = settings.CHUNK_OVERLAP

    # Clean content first
    content = clean_content(content)

    # Split into paragraphs and code blocks
    sections = split_by_sections(content)

    chunks = []
    current_chunk = ""
    current_tokens = 0
    chunk_index = 0
    start_char = 0

    for section in sections:
        section_text = section["text"]
        section_tokens = count_tokens(section_text)

        # If section alone exceeds chunk size, split it further
        if section_tokens > chunk_size:
            # Save current chunk if it has content
            if current_chunk:
                chunks.append({
                    "content": current_chunk.strip(),
                    "chunk_index": chunk_index,
                    "token_count": current_tokens,
                    "start_char": start_char,
                    "end_char": start_char + len(current_chunk),
                    "chunk_type": content_type,
                    **metadata
                })
                chunk_index += 1
                current_chunk = ""
                current_tokens = 0
                start_char += len(current_chunk)

            # Split large section into sentences/lines
            subsections = split_large_section(section_text, chunk_size, overlap)
            for subsection in subsections:
                chunks.append({
                    "content": subsection.strip(),
                    "chunk_index": chunk_index,
                    "token_count": count_tokens(subsection),
                    "start_char": start_char,
                    "end_char": start_char + len(subsection),
                    "chunk_type": content_type,
                    **metadata
                })
                chunk_index += 1
                start_char += len(subsection)

        # If adding section exceeds chunk size, start new chunk with overlap
        elif current_tokens + section_tokens > chunk_size:
            # Save current chunk
            chunks.append({
                "content": current_chunk.strip(),
                "chunk_index": chunk_index,
                "token_count": current_tokens,
                "start_char": start_char,
                "end_char": start_char + len(current_chunk),
                "chunk_type": content_type,
                **metadata
            })
            chunk_index += 1

            # Start new chunk with overlap
            overlap_text = get_overlap_text(current_chunk, overlap)
            current_chunk = overlap_text + "\n\n" + section_text
            current_tokens = count_tokens(current_chunk)
            start_char += len(current_chunk) - len(overlap_text)

        else:
            # Add section to current chunk
            if current_chunk:
                current_chunk += "\n\n" + section_text
            else:
                current_chunk = section_text
            current_tokens += section_tokens

    # Add final chunk if it has content
    if current_chunk:
        chunks.append({
            "content": current_chunk.strip(),
            "chunk_index": chunk_index,
            "token_count": current_tokens,
            "start_char": start_char,
            "end_char": start_char + len(current_chunk),
            "chunk_type": content_type,
            **metadata
        })

    return chunks


def split_by_sections(content: str) -> List[Dict[str, str]]:
    """
    Split content into sections (paragraphs, code blocks, headings)

    Args:
        content: Document content

    Returns:
        List of sections with type and text
    """
    sections = []

    # Split by code blocks first
    parts = re.split(r'(```.*?```)', content, flags=re.DOTALL)

    for part in parts:
        if not part.strip():
            continue

        # Check if it's a code block
        if part.startswith('```'):
            sections.append({"type": "code", "text": part})
        else:
            # Split text by headings
            heading_parts = re.split(r'(^#{1,6}\s+.+$)', part, flags=re.MULTILINE)

            for heading_part in heading_parts:
                if not heading_part.strip():
                    continue

                if re.match(r'^#{1,6}\s+', heading_part):
                    sections.append({"type": "heading", "text": heading_part})
                else:
                    # Split by paragraphs
                    paragraphs = re.split(r'\n\s*\n', heading_part)
                    for para in paragraphs:
                        if para.strip():
                            sections.append({"type": "paragraph", "text": para.strip()})

    return sections


def split_large_section(text: str, chunk_size: int, overlap: int) -> List[str]:
    """
    Split a large section into smaller chunks

    Args:
        text: Section text
        chunk_size: Maximum tokens per chunk
        overlap: Overlap tokens

    Returns:
        List of text chunks
    """
    chunks = []

    # Split by sentences
    sentences = re.split(r'(?<=[.!?])\s+', text)

    current_chunk = ""
    current_tokens = 0

    for sentence in sentences:
        sentence_tokens = count_tokens(sentence)

        if current_tokens + sentence_tokens > chunk_size:
            if current_chunk:
                chunks.append(current_chunk.strip())

            # Start new chunk with overlap
            overlap_text = get_overlap_text(current_chunk, overlap)
            current_chunk = overlap_text + " " + sentence
            current_tokens = count_tokens(current_chunk)
        else:
            if current_chunk:
                current_chunk += " " + sentence
            else:
                current_chunk = sentence
            current_tokens += sentence_tokens

    if current_chunk:
        chunks.append(current_chunk.strip())

    return chunks


def get_overlap_text(text: str, overlap_tokens: int) -> str:
    """
    Get the last N tokens of text for overlap

    Args:
        text: Text to get overlap from
        overlap_tokens: Number of overlap tokens

    Returns:
        Overlap text
    """
    tokens = count_tokens(text)

    if tokens <= overlap_tokens:
        return text

    # Rough estimate: get last ~overlap_tokens worth of characters
    chars_per_token = len(text) / max(tokens, 1)
    overlap_chars = int(overlap_tokens * chars_per_token)

    return text[-overlap_chars:]


def extract_metadata_from_frontmatter(content: str) -> Tuple[Dict[str, Any], str]:
    """
    Extract metadata from markdown frontmatter

    Args:
        content: Markdown content with optional frontmatter

    Returns:
        Tuple of (metadata dict, content without frontmatter)

    Frontmatter format:
        ---
        title: Introduction to ROS2
        learning_level: beginner
        ---
    """
    # TODO: Implement YAML frontmatter parsing
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
    match = re.match(frontmatter_pattern, content, re.DOTALL)

    if match:
        # Extract frontmatter (would use YAML parser in real implementation)
        frontmatter_text = match.group(1)
        content_without_frontmatter = content[match.end():]

        # Placeholder: simple key-value parsing
        metadata = {}
        for line in frontmatter_text.split('\n'):
            if ':' in line:
                key, value = line.split(':', 1)
                metadata[key.strip()] = value.strip()

        return metadata, content_without_frontmatter
    else:
        return {}, content


def extract_code_blocks(content: str) -> List[Dict[str, str]]:
    """
    Extract code blocks from markdown content

    Args:
        content: Markdown content

    Returns:
        List of code blocks with language and content

    Format:
        ```python
        code here
        ```
    """
    code_block_pattern = r'```(\w+)?\n(.*?)```'
    matches = re.findall(code_block_pattern, content, re.DOTALL)

    code_blocks = []
    for language, code in matches:
        code_blocks.append({
            "language": language or "text",
            "code": code.strip()
        })

    return code_blocks


def clean_content(content: str) -> str:
    """
    Clean and normalize content

    Args:
        content: Raw content

    Returns:
        Cleaned content

    Cleaning operations:
        - Remove excessive whitespace
        - Normalize line endings
        - Remove markdown artifacts
    """
    # Normalize line endings
    content = content.replace('\r\n', '\n')

    # Remove excessive blank lines (more than 2)
    content = re.sub(r'\n{3,}', '\n\n', content)

    # Normalize whitespace
    content = re.sub(r'[ \t]+', ' ', content)

    return content.strip()
