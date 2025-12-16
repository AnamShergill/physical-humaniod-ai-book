"""
SQLAlchemy database models for Neon PostgreSQL
"""
from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, UUID, ForeignKey, Index
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
import uuid

from app.database import Base


class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String(255), nullable=False)
    description = Column(Text)
    chapter_number = Column(Integer, nullable=False)
    total_lessons = Column(Integer, default=0)
    estimated_completion_time = Column(Integer)  # in minutes
    is_published = Column(Boolean, default=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationship
    lessons = relationship("Lesson", back_populates="chapter", lazy="select")


class Lesson(Base):
    __tablename__ = "lessons"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    title = Column(String(255), nullable=False)
    slug = Column(String(255), unique=True, nullable=False, index=True)
    content = Column(Text, nullable=False)
    chapter_id = Column(PostgresUUID(as_uuid=True), ForeignKey("chapters.id"), nullable=False)
    lesson_number = Column(Integer, nullable=False)
    estimated_reading_time = Column(Integer)  # in minutes
    is_published = Column(Boolean, default=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationship
    chapter = relationship("Chapter", back_populates="lessons")
    chunks = relationship("LessonChunk", back_populates="lesson", lazy="select")
    progress_records = relationship("UserProgress", back_populates="lesson", lazy="select")


class LessonChunk(Base):
    __tablename__ = "lesson_chunks"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    lesson_id = Column(PostgresUUID(as_uuid=True), ForeignKey("lessons.id"), nullable=False)
    embedding_id = Column(String(255), unique=True, nullable=False)  # Qdrant point ID
    content_chunk = Column(Text, nullable=False)
    chunk_order = Column(Integer, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationship
    lesson = relationship("Lesson", back_populates="chunks")

    # Index for faster queries by lesson_id
    __table_args__ = (Index('idx_lesson_chunks_lesson_id', 'lesson_id'),)


class User(Base):
    __tablename__ = "users"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    name = Column(String(255))
    language_preference = Column(String(10), default="en")
    preferences = Column(Text)  # JSON stored as text
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
    last_active = Column(DateTime(timezone=True))

    # Relationship
    progress_records = relationship("UserProgress", back_populates="user", lazy="select")


class UserProgress(Base):
    __tablename__ = "user_progress"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(PostgresUUID(as_uuid=True), ForeignKey("users.id"), nullable=False)
    lesson_id = Column(PostgresUUID(as_uuid=True), ForeignKey("lessons.id"), nullable=False)
    completion_status = Column(String(20), default="not_started")  # 'not_started', 'in_progress', 'completed'
    completion_percentage = Column(Integer, default=0)
    time_spent = Column(Integer, default=0)  # in seconds
    notes = Column(Text)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Relationships
    user = relationship("User", back_populates="progress_records")
    lesson = relationship("Lesson", back_populates="progress_records")

    # Indexes for faster queries
    __table_args__ = (
        Index('idx_user_progress_user_lesson', 'user_id', 'lesson_id', unique=True),
        Index('idx_user_progress_status', 'completion_status'),
    )


class Translation(Base):
    __tablename__ = "translations"

    id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    content_id = Column(PostgresUUID(as_uuid=True), nullable=False)  # Could be lesson_id or chapter_id
    content_type = Column(String(20), nullable=False)  # 'lesson', 'chapter'
    language_code = Column(String(10), nullable=False)
    translated_content = Column(Text)
    status = Column(String(20), default="completed")  # 'pending', 'in_progress', 'completed', 'reviewed'
    translator_id = Column(PostgresUUID(as_uuid=True))  # If tracking who translated
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    # Indexes for faster queries
    __table_args__ = (
        Index('idx_translations_content', 'content_id', 'content_type', 'language_code', unique=True),
        Index('idx_translations_language', 'language_code'),
        Index('idx_translations_status', 'status'),
    )


# Create indexes for better performance
def create_indexes(engine):
    """
    Create database indexes for better performance
    """
    from sqlalchemy import text

    indexes = [
        # Chapters
        "CREATE INDEX IF NOT EXISTS idx_chapters_published ON chapters (is_published);",
        "CREATE INDEX IF NOT EXISTS idx_chapters_number ON chapters (chapter_number);",

        # Lessons
        "CREATE INDEX IF NOT EXISTS idx_lessons_published ON lessons (is_published);",
        "CREATE INDEX IF NOT EXISTS idx_lessons_chapter ON lessons (chapter_id);",
        "CREATE INDEX IF NOT EXISTS idx_lessons_number ON lessons (chapter_id, lesson_number);",

        # Users
        "CREATE INDEX IF NOT EXISTS idx_users_email ON users (email);",
        "CREATE INDEX IF NOT EXISTS idx_users_language ON users (language_preference);",

        # Lesson chunks (already has index above)

        # User progress (already has indexes above)
    ]

    with engine.connect() as conn:
        for index_sql in indexes:
            conn.execute(text(index_sql))
        conn.commit()