from typing import List, Optional
from uuid import UUID, uuid4
from datetime import datetime
import logging

from app.models.lesson import Lesson, LessonCreate, LessonUpdate, Chapter, ChapterCreate, LessonChunk, LessonChunkCreate
from app.config import settings

logger = logging.getLogger(__name__)


class LessonService:
    def __init__(self):
        # In a real implementation, this would connect to Neon database
        self.lessons = {}
        self.chapters = {}
        self.lesson_chunks = {}

    async def get_lesson(self, lesson_id: UUID) -> Optional[Lesson]:
        """Get a lesson by ID"""
        return self.lessons.get(lesson_id)

    async def get_lesson_by_slug(self, slug: str) -> Optional[Lesson]:
        """Get a lesson by slug"""
        for lesson in self.lessons.values():
            if lesson.slug == slug:
                return lesson
        return None

    async def get_lessons(self, skip: int = 0, limit: int = 100) -> List[Lesson]:
        """Get multiple lessons with pagination"""
        all_lessons = list(self.lessons.values())
        return all_lessons[skip:skip + limit]

    async def create_lesson(self, lesson_data: LessonCreate) -> Lesson:
        """Create a new lesson"""
        lesson_id = uuid4()
        lesson = Lesson(
            id=lesson_id,
            **lesson_data.dict(),
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )
        self.lessons[lesson_id] = lesson
        return lesson

    async def update_lesson(self, lesson_id: UUID, lesson_data: LessonUpdate) -> Optional[Lesson]:
        """Update an existing lesson"""
        if lesson_id not in self.lessons:
            return None

        lesson = self.lessons[lesson_id]
        update_data = lesson_data.dict(exclude_unset=True)
        for field, value in update_data.items():
            setattr(lesson, field, value)
        lesson.updated_at = datetime.utcnow()
        self.lessons[lesson_id] = lesson
        return lesson

    async def delete_lesson(self, lesson_id: UUID) -> bool:
        """Delete a lesson"""
        if lesson_id in self.lessons:
            del self.lessons[lesson_id]
            # Also delete associated chunks
            chunks_to_delete = [chunk_id for chunk_id, chunk in self.lesson_chunks.items()
                               if chunk.lesson_id == lesson_id]
            for chunk_id in chunks_to_delete:
                del self.lesson_chunks[chunk_id]
            return True
        return False

    async def get_chapter(self, chapter_id: UUID) -> Optional[Chapter]:
        """Get a chapter by ID"""
        return self.chapters.get(chapter_id)

    async def get_chapters(self) -> List[Chapter]:
        """Get all chapters"""
        return list(self.chapters.values())

    async def create_chapter(self, chapter_data: ChapterCreate) -> Chapter:
        """Create a new chapter"""
        chapter_id = uuid4()
        chapter = Chapter(
            id=chapter_id,
            **chapter_data.dict(),
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow(),
            lessons=[]
        )
        self.chapters[chapter_id] = chapter
        return chapter

    async def get_lessons_by_chapter(self, chapter_id: UUID) -> List[Lesson]:
        """Get all lessons in a chapter"""
        chapter_lessons = []
        for lesson in self.lessons.values():
            if lesson.chapter_id == chapter_id:
                chapter_lessons.append(lesson)
        # Sort by lesson number
        chapter_lessons.sort(key=lambda x: x.lesson_number)
        return chapter_lessons

    async def chunk_lesson_content(self, lesson_id: UUID) -> List[LessonChunk]:
        """Chunk lesson content for embedding"""
        lesson = await self.get_lesson(lesson_id)
        if not lesson:
            return []

        # Simple chunking by character count
        content = lesson.content
        chunks = []
        chunk_size = settings.LESSON_CHUNK_SIZE
        overlap = settings.LESSON_CHUNK_OVERLAP

        start = 0
        chunk_order = 0

        while start < len(content):
            end = start + chunk_size
            chunk_content = content[start:end]

            # Create embedding ID
            embedding_id = f"lesson_{lesson_id}_chunk_{chunk_order}"

            chunk = LessonChunkCreate(
                lesson_id=lesson_id,
                embedding_id=embedding_id,
                content_chunk=chunk_content,
                chunk_order=chunk_order
            )

            # In real implementation, would save to database
            chunk_obj = LessonChunk(
                id=uuid4(),
                **chunk.dict(),
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow()
            )
            self.lesson_chunks[chunk_obj.id] = chunk_obj
            chunks.append(chunk_obj)

            start = end - overlap
            chunk_order += 1

        return chunks

    async def get_lesson_chunks(self, lesson_id: UUID) -> List[LessonChunk]:
        """Get all chunks for a lesson"""
        lesson_chunks = []
        for chunk in self.lesson_chunks.values():
            if chunk.lesson_id == lesson_id:
                lesson_chunks.append(chunk)
        # Sort by chunk order
        lesson_chunks.sort(key=lambda x: x.chunk_order)
        return lesson_chunks


# Global service instance
lesson_service = LessonService()