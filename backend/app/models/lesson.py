from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime
from uuid import UUID, uuid4


class LessonBase(BaseModel):
    title: str
    slug: str
    content: str
    chapter_id: UUID
    lesson_number: int
    estimated_reading_time: int  # in minutes
    is_published: bool = True


class LessonCreate(LessonBase):
    pass


class LessonUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None
    lesson_number: Optional[int] = None
    estimated_reading_time: Optional[int] = None
    is_published: Optional[bool] = None


class Lesson(LessonBase):
    id: UUID
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ChapterBase(BaseModel):
    title: str
    description: str
    chapter_number: int
    total_lessons: int
    estimated_completion_time: int  # in minutes
    is_published: bool = True


class ChapterCreate(ChapterBase):
    pass


class ChapterUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    chapter_number: Optional[int] = None
    estimated_completion_time: Optional[int] = None
    is_published: Optional[bool] = None


class Chapter(ChapterBase):
    id: UUID
    created_at: datetime
    updated_at: datetime
    lessons: List[Lesson] = []

    class Config:
        from_attributes = True


class LessonChunkBase(BaseModel):
    lesson_id: UUID
    embedding_id: str  # Qdrant point ID
    content_chunk: str
    chunk_order: int


class LessonChunkCreate(LessonChunkBase):
    pass


class LessonChunk(LessonChunkBase):
    id: UUID
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True