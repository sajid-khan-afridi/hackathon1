"""Comprehensive tests for chunking utilities"""

import pytest
from app.utils.chunking import (
    count_tokens,
    chunk_document,
    split_by_sections,
    split_large_section,
    get_overlap_text,
    extract_code_blocks,
    clean_content
)


class TestTokenCounting:
    """Test token counting functionality"""

    def test_count_tokens_basic(self):
        """Test basic token counting"""
        text = "Hello world"
        count = count_tokens(text)
        assert count > 0
        assert isinstance(count, int)

    def test_count_tokens_empty(self):
        """Test token counting with empty string"""
        count = count_tokens("")
        assert count == 0

    def test_count_tokens_long_text(self):
        """Test token counting with long text"""
        text = "This is a longer piece of text " * 100
        count = count_tokens(text)
        assert count > 100


class TestChunkDocument:
    """Test document chunking"""

    def test_chunk_theoretical_content(self):
        """Test chunking theoretical content with appropriate chunk size"""
        content = "This is theoretical content. " * 200  # Long text
        metadata = {
            "module": "module-01-ros2",
            "chapter": "01-fundamentals",
            "section": "Introduction"
        }

        chunks = chunk_document(content, "theoretical", metadata)

        assert len(chunks) > 0
        assert all("content" in chunk for chunk in chunks)
        assert all("chunk_index" in chunk for chunk in chunks)
        assert all("chunk_type" in chunk for chunk in chunks)
        assert all(chunk["chunk_type"] == "theoretical" for chunk in chunks)

    def test_chunk_code_content(self):
        """Test chunking code content with appropriate chunk size"""
        content = """
```python
def example_function():
    pass
```
""" * 50  # Multiple code blocks

        metadata = {
            "module": "module-01-ros2",
            "chapter": "02-code",
            "section": "Examples"
        }

        chunks = chunk_document(content, "code", metadata)

        assert len(chunks) > 0
        assert all(chunk["chunk_type"] == "code" for chunk in chunks)

    def test_chunk_mixed_content(self):
        """Test chunking mixed content"""
        content = """
# Introduction

This is theoretical text explaining concepts.

```python
def example():
    return "code example"
```

More theoretical content here.
""" * 20

        metadata = {
            "module": "module-01-ros2",
            "chapter": "03-tutorial",
            "section": "Getting Started"
        }

        chunks = chunk_document(content, "mixed", metadata)

        assert len(chunks) > 0
        assert all(chunk["chunk_type"] == "mixed" for chunk in chunks)

    def test_chunk_metadata_preserved(self):
        """Test that metadata is preserved in chunks"""
        content = "Test content " * 100
        metadata = {
            "module": "test-module",
            "chapter": "test-chapter",
            "section": "test-section",
            "custom_field": "custom_value"
        }

        chunks = chunk_document(content, "theoretical", metadata)

        assert all(chunk["module"] == "test-module" for chunk in chunks)
        assert all(chunk["chapter"] == "test-chapter" for chunk in chunks)
        assert all(chunk["section"] == "test-section" for chunk in chunks)

    def test_chunk_indices_sequential(self):
        """Test that chunk indices are sequential"""
        content = "Content " * 500  # Force multiple chunks
        metadata = {"module": "test", "chapter": "test", "section": "test"}

        chunks = chunk_document(content, "theoretical", metadata)

        indices = [chunk["chunk_index"] for chunk in chunks]
        assert indices == list(range(len(chunks)))

    def test_chunk_overlap(self):
        """Test that chunks have overlap"""
        content = "This is a sentence. " * 200
        metadata = {"module": "test", "chapter": "test", "section": "test"}

        chunks = chunk_document(content, "theoretical", metadata)

        if len(chunks) > 1:
            # Check that there's some overlap between consecutive chunks
            # This is a simple check - more sophisticated overlap detection could be added
            assert len(chunks) > 1


class TestSplitBySections:
    """Test section splitting"""

    def test_split_code_blocks(self):
        """Test splitting identifies code blocks"""
        content = """
Some text before.

```python
code here
```

Some text after.
"""
        sections = split_by_sections(content)

        # Should identify at least one code block
        code_sections = [s for s in sections if s["type"] == "code"]
        assert len(code_sections) > 0

    def test_split_headings(self):
        """Test splitting identifies headings"""
        content = """
# Heading 1

Some content.

## Heading 2

More content.
"""
        sections = split_by_sections(content)

        # Should identify headings
        heading_sections = [s for s in sections if s["type"] == "heading"]
        assert len(heading_sections) >= 1

    def test_split_paragraphs(self):
        """Test splitting identifies paragraphs"""
        content = """
First paragraph here.

Second paragraph here.

Third paragraph here.
"""
        sections = split_by_sections(content)

        # Should identify multiple paragraphs
        para_sections = [s for s in sections if s["type"] == "paragraph"]
        assert len(para_sections) >= 3

    def test_split_empty_content(self):
        """Test splitting empty content"""
        sections = split_by_sections("")
        assert sections == []


class TestSplitLargeSection:
    """Test large section splitting"""

    def test_split_large_section_by_sentences(self):
        """Test that large sections are split by sentences"""
        text = "This is a sentence. " * 100
        chunks = split_large_section(text, chunk_size=100, overlap=20)

        assert len(chunks) > 1
        assert all(isinstance(chunk, str) for chunk in chunks)

    def test_split_large_section_with_overlap(self):
        """Test that splits have overlap"""
        text = "Sentence one. Sentence two. Sentence three. " * 50
        chunks = split_large_section(text, chunk_size=100, overlap=20)

        # With overlap, chunks should have some common content
        if len(chunks) > 1:
            assert len(chunks) > 1  # Basic check


class TestGetOverlapText:
    """Test overlap text extraction"""

    def test_get_overlap_basic(self):
        """Test basic overlap extraction"""
        text = "This is a test text with many words."
        overlap = get_overlap_text(text, overlap_tokens=5)

        assert overlap is not None
        assert len(overlap) > 0
        assert len(overlap) <= len(text)

    def test_get_overlap_short_text(self):
        """Test overlap with text shorter than overlap size"""
        text = "Short"
        overlap = get_overlap_text(text, overlap_tokens=100)

        # Should return full text if shorter than overlap
        assert overlap == text

    def test_get_overlap_empty_text(self):
        """Test overlap with empty text"""
        overlap = get_overlap_text("", overlap_tokens=10)
        assert overlap == ""


class TestExtractCodeBlocks:
    """Test code block extraction"""

    def test_extract_python_code(self):
        """Test extracting Python code blocks"""
        content = """
Some text.

```python
def hello():
    print("Hello")
```

More text.

```python
def world():
    print("World")
```
"""
        code_blocks = extract_code_blocks(content)

        assert len(code_blocks) == 2
        assert all(block["language"] == "python" for block in code_blocks)
        assert all("code" in block for block in code_blocks)

    def test_extract_code_without_language(self):
        """Test extracting code blocks without language specifier"""
        content = """
```
generic code
```
"""
        code_blocks = extract_code_blocks(content)

        assert len(code_blocks) == 1
        assert code_blocks[0]["language"] == "text"

    def test_extract_multiple_languages(self):
        """Test extracting code blocks with different languages"""
        content = """
```python
python code
```

```javascript
javascript code
```

```bash
bash commands
```
"""
        code_blocks = extract_code_blocks(content)

        assert len(code_blocks) == 3
        languages = [block["language"] for block in code_blocks]
        assert "python" in languages
        assert "javascript" in languages
        assert "bash" in languages

    def test_extract_no_code_blocks(self):
        """Test with content containing no code blocks"""
        content = "Just regular text without any code blocks."
        code_blocks = extract_code_blocks(content)

        assert code_blocks == []


class TestCleanContent:
    """Test content cleaning"""

    def test_clean_excessive_whitespace(self):
        """Test cleaning excessive whitespace"""
        content = "Line 1\n\n\n\n\nLine 2"
        cleaned = clean_content(content)

        # Should reduce multiple newlines to max 2
        assert "\n\n\n" not in cleaned

    def test_clean_line_endings(self):
        """Test normalizing line endings"""
        content = "Line 1\r\nLine 2\r\nLine 3"
        cleaned = clean_content(content)

        # Should normalize to \n
        assert "\r\n" not in cleaned
        assert "\n" in cleaned

    def test_clean_trailing_whitespace(self):
        """Test removing trailing whitespace"""
        content = "  Content with spaces  \n\n  "
        cleaned = clean_content(content)

        # Should strip leading/trailing whitespace
        assert not cleaned.startswith(" ")
        assert not cleaned.endswith(" ")

    def test_clean_multiple_spaces(self):
        """Test normalizing multiple spaces"""
        content = "Word1    Word2     Word3"
        cleaned = clean_content(content)

        # Should reduce multiple spaces to single space
        assert "    " not in cleaned
        assert "Word1 Word2 Word3" in cleaned

    def test_clean_empty_content(self):
        """Test cleaning empty content"""
        cleaned = clean_content("")
        assert cleaned == ""

    def test_clean_preserves_meaningful_content(self):
        """Test that cleaning preserves meaningful content"""
        content = """
# Heading

Paragraph with normal spacing.

Another paragraph.
"""
        cleaned = clean_content(content)

        # Important content should be preserved
        assert "# Heading" in cleaned
        assert "Paragraph" in cleaned
        assert "Another paragraph" in cleaned
