"""
Script to import existing textbook content into Neon database and Qdrant vector store
"""
import asyncio
import os
import re
from pathlib import Path
from uuid import UUID, uuid4
from datetime import datetime
import logging

from sqlalchemy.orm import Session
from sqlalchemy import create_engine
from sqlalchemy.dialects.postgresql import insert

# Add the backend to the Python path
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from app.config import settings
from app.database import engine, SessionLocal
from app.models.db_models import Chapter, Lesson, LessonChunk, User, UserProgress
from app.services.lesson_service import lesson_service
from app.services.rag_service import rag_service
from rag.lesson_chunker import lesson_chunker

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def slugify(text: str) -> str:
    """
    Convert text to URL-friendly slug
    """
    import re
    import unicodedata

    # Normalize unicode characters
    text = unicodedata.normalize('NFKD', text)
    text = text.encode('ascii', 'ignore').decode('ascii')

    # Convert to lowercase and replace spaces with hyphens
    text = re.sub(r'[^\w\s-]', '', text).strip().lower()
    text = re.sub(r'[-\s]+', '-', text)

    return text


def extract_lesson_metadata(content: str) -> dict:
    """
    Extract metadata from lesson content
    """
    # Extract title from first H1
    title_match = re.search(r'^#\s+(.+)', content, re.MULTILINE)
    title = title_match.group(1) if title_match else "Untitled Lesson"

    # Count words for estimated reading time (225 words per minute for technical content)
    words = len(re.findall(r'\b\w+\b', content))
    estimated_reading_time = max(1, words // 225)  # At least 1 minute

    # Extract learning objectives
    objectives_match = re.search(r'## Learning Objectives\s+((?:-\s.*\n?)*)', content)
    objectives = []
    if objectives_match:
        objectives_text = objectives_match.group(1)
        objectives = [obj.strip('- \n') for obj in objectives_text.split('\n') if obj.strip()]

    return {
        'title': title,
        'estimated_reading_time': estimated_reading_time,
        'objectives_count': len(objectives)
    }


def import_lessons_from_docs(docs_path: str, db: Session):
    """
    Import lessons from the documentation structure into the database
    """
    docs_path = Path(docs_path)

    # Find all chapter directories
    chapter_dirs = [d for d in docs_path.iterdir() if d.is_dir() and d.name.startswith('0')]

    for chapter_dir in sorted(chapter_dirs):
        logger.info(f"Processing chapter: {chapter_dir.name}")

        # Extract chapter number and title from directory name
        chapter_num = int(chapter_dir.name.split('-')[0])
        chapter_title = ' '.join(chapter_dir.name.split('-')[1:]).replace('_', ' ').title()

        # Create or get chapter
        chapter = db.query(Chapter).filter(Chapter.chapter_number == chapter_num).first()
        if not chapter:
            chapter = Chapter(
                title=chapter_title,
                description=f"Chapter {chapter_num} of the Physical AI & Humanoid Robotics Textbook",
                chapter_number=chapter_num,
                total_lessons=0,
                estimated_completion_time=0
            )
            db.add(chapter)
            db.commit()
            db.refresh(chapter)

        # Find all lesson files in this chapter
        lesson_files = list(chapter_dir.glob("*.md"))

        for i, lesson_file in enumerate(sorted(lesson_files)):
            logger.info(f"  Processing lesson: {lesson_file.name}")

            # Read lesson content
            with open(lesson_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract metadata
            metadata = extract_lesson_metadata(content)

            # Create slug from filename
            slug = slugify(lesson_file.stem)

            # Check if lesson already exists
            existing_lesson = db.query(Lesson).filter(Lesson.slug == slug).first()
            if existing_lesson:
                logger.info(f"    Lesson {slug} already exists, skipping...")
                continue

            # Create lesson
            lesson = Lesson(
                title=metadata['title'],
                slug=slug,
                content=content,
                chapter_id=chapter.id,
                lesson_number=i + 1,
                estimated_reading_time=metadata['estimated_reading_time'],
                is_published=True
            )

            db.add(lesson)
            db.commit()
            db.refresh(lesson)

            logger.info(f"    Created lesson: {lesson.title}")

            # Chunk and index the lesson content
            try:
                # Use the lesson service to chunk the content
                chunks = asyncio.run(lesson_service.chunk_lesson_content(lesson.id))

                # Index the chunks in Qdrant
                asyncio.run(rag_service.index_lesson_chunks(chunks))

                logger.info(f"    Indexed {len(chunks)} chunks for lesson {lesson.title}")
            except Exception as e:
                logger.error(f"    Error chunking/indexing lesson {lesson.title}: {e}")

        # Update total lessons in chapter
        chapter.total_lessons = len(lesson_files)
        db.commit()

        logger.info(f"Completed chapter {chapter_num}: {chapter.title} with {len(lesson_files)} lessons")


def main():
    """
    Main function to import content
    """
    logger.info("Starting content import process...")

    # Create database session
    db = SessionLocal()

    try:
        # Import lessons from website docs
        website_docs_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "website", "docs", "chapters")
        if os.path.exists(website_docs_path):
            import_lessons_from_docs(website_docs_path, db)
        else:
            logger.warning(f"Website docs path not found: {website_docs_path}")

        logger.info("Content import completed successfully!")

    except Exception as e:
        logger.error(f"Error during content import: {e}")
        db.rollback()
        raise
    finally:
        db.close()


if __name__ == "__main__":
    main()