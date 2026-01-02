"""
Script to load textbook content from markdown files and index it in the RAG system
"""
import asyncio
import os
from pathlib import Path
from typing import List
from uuid import uuid4
from datetime import datetime

from app.services.lesson_service import lesson_service
from app.services.rag_service import rag_service
from app.models.lesson import LessonChunkCreate, LessonChunk


async def load_textbook_content():
    """
    Load textbook content from docs directory and index it in the RAG system
    """
    print("Starting textbook content loading...")

    # Get all markdown files from the chapters directory
    docs_path = Path("C:/Users/Bruno/Desktop/AI-BOOK/docs/chapters")
    markdown_files = list(docs_path.rglob("*.md"))

    print(f"Found {len(markdown_files)} markdown files")

    total_chunks = 0

    for file_path in markdown_files:
        print(f"Processing file: {file_path}")

        try:
            # Read the content of the markdown file
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            if not content.strip():
                print(f"  Skipping empty file: {file_path}")
                continue

            # Create a lesson ID based on the file path
            # Convert path to a consistent format for lesson identification
            relative_path = file_path.relative_to(docs_path)
            lesson_slug = str(relative_path).replace('\\', '/').replace('.md', '').replace('/', '-')

            print(f"  Creating lesson with slug: {lesson_slug}")

            # Create a lesson object (in memory since we're using in-memory storage)
            lesson_id = uuid4()

            # Create chunks from the content
            chunks = await create_chunks_from_content(lesson_id, content)

            if chunks:
                # Index the chunks in the RAG system
                await rag_service.index_lesson_chunks(chunks)
                print(f"  Indexed {len(chunks)} chunks for lesson {lesson_slug}")
                total_chunks += len(chunks)
            else:
                print(f"  No chunks created for lesson {lesson_slug}")

        except Exception as e:
            print(f"  Error processing file {file_path}: {str(e)}")
            import traceback
            print(f"  Traceback: {traceback.format_exc()}")

    print(f"Content loading completed. Total chunks indexed: {total_chunks}")

    # Verify the indexing worked by checking the collection
    try:
        collection_info = rag_service.qdrant_client.get_collection(rag_service.collection_name)
        print(f"Collection '{rag_service.collection_name}' has {collection_info.points_count} vectors")
    except Exception as e:
        print(f"Error getting collection info: {str(e)}")


async def create_chunks_from_content(lesson_id, content: str) -> List[LessonChunk]:
    """
    Create chunks from content following the same logic as lesson_service
    """
    # Simple chunking by character count
    chunks = []
    chunk_size = 1000  # Using the same size as settings.LESSON_CHUNK_SIZE
    overlap = 100  # Using the same overlap as settings.LESSON_CHUNK_OVERLAP

    start = 0
    chunk_order = 0

    while start < len(content):
        end = start + chunk_size
        chunk_content = content[start:end]

        # Create embedding ID - use the chunk's actual ID which is already a UUID
        embedding_id = str(uuid4())  # We'll use the chunk_obj.id for Qdrant instead

        chunk = LessonChunkCreate(
            lesson_id=lesson_id,
            embedding_id=embedding_id,
            content_chunk=chunk_content,
            chunk_order=chunk_order
        )

        # Create the full chunk object
        chunk_obj = LessonChunk(
            id=uuid4(),
            **chunk.dict(),
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )

        # Store in lesson service memory (though this is mainly for the RAG service)
        lesson_service.lesson_chunks[chunk_obj.id] = chunk_obj
        chunks.append(chunk_obj)

        start = end - overlap
        chunk_order += 1

    return chunks


if __name__ == "__main__":
    asyncio.run(load_textbook_content())