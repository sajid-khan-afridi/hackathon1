"""Document ingestion pipeline for Physical AI book content

Complete implementation:
1. Scan for markdown/MDX files
2. Extract metadata from frontmatter
3. Chunk content using variable-size strategy
4. Generate embeddings with OpenAI
5. Store in Postgres + Qdrant with atomic dual-write
"""

import asyncio
import sys
import argparse
import logging
from pathlib import Path
from typing import List, Dict, Any, Optional
import uuid

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.core.config import settings
from app.utils.chunking import chunk_document, extract_metadata_from_frontmatter, clean_content
from app.services.storage import storage_service
from app.models.database import Document
from openai import AsyncOpenAI

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def generate_embedding(text: str, client: AsyncOpenAI) -> List[float]:
    """
    Generate embedding for text using OpenAI

    Args:
        text: Text to embed
        client: OpenAI async client

    Returns:
        Embedding vector (1536-dim)
    """
    try:
        response = await client.embeddings.create(
            model=settings.OPENAI_EMBEDDING_MODEL,
            input=text
        )
        return response.data[0].embedding
    except Exception as e:
        logger.error(f"Failed to generate embedding: {e}")
        raise


async def ingest_document_file(
    md_file: Path,
    docs_directory: Path,
    openai_client: AsyncOpenAI,
    dry_run: bool = False
) -> Dict[str, Any]:
    """
    Ingest a single document file

    Args:
        md_file: Path to markdown file
        docs_directory: Base docs directory
        openai_client: OpenAI async client
        dry_run: If True, don't actually insert into databases

    Returns:
        Ingestion statistics
    """
    stats = {
        "file": str(md_file.relative_to(docs_directory)),
        "chunks_created": 0,
        "tokens_processed": 0,
        "success": False,
        "error": None
    }

    try:
        logger.info(f"Processing: {md_file.relative_to(docs_directory)}")

        # Read file content
        content = md_file.read_text(encoding="utf-8")

        # Extract metadata from frontmatter
        metadata, content_clean = extract_metadata_from_frontmatter(content)

        # Derive module/chapter from path
        relative_path = md_file.relative_to(docs_directory)
        path_parts = relative_path.parts

        if len(path_parts) >= 1:
            module = path_parts[0]  # e.g., 'module-01-ros2'
            chapter = path_parts[1] if len(path_parts) > 1 else None
            section = path_parts[2] if len(path_parts) > 2 else None
        else:
            module = "general"
            chapter = None
            section = None

        # Extract filename without extension
        filename = md_file.stem

        # Determine content type
        content_type = metadata.get("content_type", "tutorial")
        if not metadata.get("content_type"):
            if "```" in content:
                content_type = "code" if content.count("```") > 3 else "tutorial"
            else:
                content_type = "theoretical"

        # Extract title
        title = metadata.get("title", filename.replace("-", " ").title())

        # Extract learning level
        learning_level = metadata.get("learning_level", "beginner")

        # Extract frameworks
        frameworks_raw = metadata.get("frameworks", [])
        if isinstance(frameworks_raw, str):
            frameworks = [f.strip() for f in frameworks_raw.split(",")]
        else:
            frameworks = frameworks_raw

        if dry_run:
            logger.info(f"  [DRY RUN] Would create document: {title}")
            logger.info(f"  Module: {module}, Chapter: {chapter}, Type: {content_type}")

            # Chunk document (dry run)
            chunks = chunk_document(
                content_clean,
                content_type,
                {
                    "module": module,
                    "chapter": chapter,
                    "section": section,
                    "learning_level": learning_level,
                    "frameworks": frameworks
                }
            )
            logger.info(f"  Would create {len(chunks)} chunks")
            stats["chunks_created"] = len(chunks)
            stats["tokens_processed"] = sum(chunk["token_count"] for chunk in chunks)
            stats["success"] = True
            return stats

        # Create document in database
        async with storage_service.get_session() as session:
            document = Document(
                title=title,
                module=module,
                chapter=chapter,
                section=section,
                content_type=content_type,
                file_path=str(relative_path),
                author=metadata.get("author"),
                tags=metadata.get("tags", []),
                prerequisites=metadata.get("prerequisites", []),
                learning_level=learning_level,
                frameworks={"frameworks": frameworks} if frameworks else None
            )

            session.add(document)
            await session.flush()  # Get document ID
            document_id = document.id

            # Chunk document
            chunks = chunk_document(
                content_clean,
                content_type,
                {
                    "module": module,
                    "chapter": chapter,
                    "section": section,
                    "learning_level": learning_level,
                    "frameworks": frameworks
                }
            )

            logger.info(f"  Created {len(chunks)} chunks")
            stats["chunks_created"] = len(chunks)

            # Process each chunk
            for chunk in chunks:
                try:
                    # Generate embedding
                    embedding = await generate_embedding(chunk["content"], openai_client)

                    # Insert chunk with vector
                    chunk_id = await storage_service.insert_chunk_with_vector(
                        content=chunk["content"],
                        embedding=embedding,
                        document_id=str(document_id),
                        chunk_index=chunk["chunk_index"],
                        metadata={
                            "token_count": chunk["token_count"],
                            "start_char": chunk.get("start_char"),
                            "end_char": chunk.get("end_char"),
                            "module": module,
                            "chapter": chapter,
                            "section": section,
                            "chunk_type": chunk["chunk_type"],
                            "learning_level": learning_level,
                            "frameworks": frameworks
                        }
                    )

                    if chunk_id:
                        stats["tokens_processed"] += chunk["token_count"]
                    else:
                        logger.warning(f"  Failed to insert chunk {chunk['chunk_index']}")

                except Exception as chunk_error:
                    logger.error(f"  Error processing chunk {chunk['chunk_index']}: {chunk_error}")
                    continue

            await session.commit()

        logger.info(f"  Successfully ingested {md_file.name}")
        logger.info(f"  - Document ID: {document_id}")
        logger.info(f"  - Chunks: {stats['chunks_created']}")
        logger.info(f"  - Tokens: {stats['tokens_processed']}")

        stats["success"] = True

    except Exception as e:
        logger.error(f"  Error processing {md_file.name}: {e}")
        import traceback
        traceback.print_exc()
        stats["error"] = str(e)

    return stats


async def ingest_documents(
    docs_directory: Path,
    file_pattern: Optional[str] = None,
    dry_run: bool = False
):
    """
    Ingest documents from docs directory

    Args:
        docs_directory: Path to docs/ directory
        file_pattern: Optional glob pattern to filter files
        dry_run: If True, don't actually insert into databases
    """
    logger.info("="*60)
    logger.info("Physical AI RAG - Document Ingestion Pipeline")
    logger.info("="*60)
    logger.info(f"Docs directory: {docs_directory}")
    logger.info(f"Dry run: {dry_run}")

    # Initialize OpenAI client
    openai_client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)

    # Find all markdown files
    if file_pattern:
        markdown_files = list(docs_directory.glob(file_pattern))
    else:
        markdown_files = list(docs_directory.glob("**/*.md")) + list(docs_directory.glob("**/*.mdx"))

    # Filter out certain files
    markdown_files = [f for f in markdown_files if not any(
        skip in str(f) for skip in ["node_modules", ".git", "README.md"]
    )]

    logger.info(f"Found {len(markdown_files)} markdown files\n")

    # Process all files
    all_stats = []
    for md_file in markdown_files:
        stats = await ingest_document_file(md_file, docs_directory, openai_client, dry_run)
        all_stats.append(stats)

    # Print summary
    logger.info("\n" + "="*60)
    logger.info("INGESTION SUMMARY")
    logger.info("="*60)

    successful = [s for s in all_stats if s["success"]]
    failed = [s for s in all_stats if not s["success"]]

    logger.info(f"\nTotal files processed: {len(all_stats)}")
    logger.info(f"Successful: {len(successful)}")
    logger.info(f"Failed: {len(failed)}")

    if successful:
        total_chunks = sum(s["chunks_created"] for s in successful)
        total_tokens = sum(s["tokens_processed"] for s in successful)
        logger.info(f"\nTotal chunks created: {total_chunks}")
        logger.info(f"Total tokens processed: {total_tokens}")

    if failed:
        logger.info("\nFailed files:")
        for s in failed:
            logger.info(f"  - {s['file']}: {s['error']}")

    logger.info("\n" + "="*60)

    # Close connections
    await storage_service.close()


async def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Ingest Physical AI book documents")
    parser.add_argument(
        "--file",
        type=str,
        help="Specific file to ingest (relative to docs directory)"
    )
    parser.add_argument(
        "--pattern",
        type=str,
        help="Glob pattern to filter files (e.g., 'module-01-ros2/**/*.md')"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Dry run mode - don't actually insert into databases"
    )

    args = parser.parse_args()

    # Get docs directory from project root
    project_root = Path(__file__).parent.parent.parent
    docs_dir = project_root / "docs"

    if not docs_dir.exists():
        logger.error(f"Error: docs directory not found at {docs_dir}")
        sys.exit(1)

    # Determine file pattern
    if args.file:
        file_pattern = args.file
    elif args.pattern:
        file_pattern = args.pattern
    else:
        file_pattern = None

    await ingest_documents(docs_dir, file_pattern, args.dry_run)


if __name__ == "__main__":
    asyncio.run(main())
