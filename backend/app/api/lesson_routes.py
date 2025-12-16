from fastapi import APIRouter, HTTPException, Query
from typing import List, Optional
from uuid import UUID

from app.models.lesson import Lesson, LessonCreate, LessonUpdate, Chapter, ChapterCreate, LessonChunk
from app.services.lesson_service import lesson_service

router = APIRouter()


@router.get("/lessons", response_model=List[Lesson])
async def get_lessons(
    skip: int = Query(0, ge=0),
    limit: int = Query(100, ge=1, le=1000),
    chapter_id: Optional[UUID] = Query(None)
):
    """Get all lessons with optional filtering"""
    if chapter_id:
        lessons = await lesson_service.get_lessons_by_chapter(chapter_id)
        # Apply pagination manually
        start = skip
        end = start + limit
        return lessons[start:end]
    else:
        return await lesson_service.get_lessons(skip=skip, limit=limit)


@router.get("/lessons/{lesson_id}", response_model=Lesson)
async def get_lesson(lesson_id: UUID):
    """Get a specific lesson by ID"""
    lesson = await lesson_service.get_lesson(lesson_id)
    if not lesson:
        raise HTTPException(status_code=404, detail="Lesson not found")
    return lesson


@router.get("/lessons/slug/{slug}", response_model=Lesson)
async def get_lesson_by_slug(slug: str):
    """Get a specific lesson by slug"""
    lesson = await lesson_service.get_lesson_by_slug(slug)
    if not lesson:
        raise HTTPException(status_code=404, detail="Lesson not found")
    return lesson


@router.post("/lessons", response_model=Lesson)
async def create_lesson(lesson_data: LessonCreate):
    """Create a new lesson"""
    return await lesson_service.create_lesson(lesson_data)


@router.put("/lessons/{lesson_id}", response_model=Lesson)
async def update_lesson(lesson_id: UUID, lesson_data: LessonUpdate):
    """Update an existing lesson"""
    lesson = await lesson_service.update_lesson(lesson_id, lesson_data)
    if not lesson:
        raise HTTPException(status_code=404, detail="Lesson not found")
    return lesson


@router.delete("/lessons/{lesson_id}")
async def delete_lesson(lesson_id: UUID):
    """Delete a lesson"""
    success = await lesson_service.delete_lesson(lesson_id)
    if not success:
        raise HTTPException(status_code=404, detail="Lesson not found")
    return {"message": "Lesson deleted successfully"}


@router.get("/chapters", response_model=List[Chapter])
async def get_chapters():
    """Get all chapters"""
    return await lesson_service.get_chapters()


@router.get("/chapters/{chapter_id}", response_model=Chapter)
async def get_chapter(chapter_id: UUID):
    """Get a specific chapter by ID"""
    chapter = await lesson_service.get_chapter(chapter_id)
    if not chapter:
        raise HTTPException(status_code=404, detail="Chapter not found")
    return chapter


@router.post("/chapters", response_model=Chapter)
async def create_chapter(chapter_data: ChapterCreate):
    """Create a new chapter"""
    return await lesson_service.create_chapter(chapter_data)


@router.get("/lessons/{lesson_id}/chunks", response_model=List[LessonChunk])
async def get_lesson_chunks(lesson_id: UUID):
    """Get all chunks for a lesson"""
    chunks = await lesson_service.get_lesson_chunks(lesson_id)
    return chunks


@router.post("/lessons/{lesson_id}/chunk")
async def chunk_and_index_lesson(lesson_id: UUID):
    """Chunk lesson content and prepare for embedding"""
    chunks = await lesson_service.chunk_lesson_content(lesson_id)
    return {
        "message": f"Lesson chunked into {len(chunks)} parts",
        "chunks_created": len(chunks)
    }