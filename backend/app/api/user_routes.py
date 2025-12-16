from fastapi import APIRouter, HTTPException, Depends
from typing import List, Dict
from uuid import UUID

from app.models.user import User, UserCreate, UserUpdate, UserProgress, UserProgressCreate, UserProgressUpdate
from app.services.user_service import user_service

router = APIRouter()


@router.get("/users/{user_id}", response_model=User)
async def get_user(user_id: UUID):
    """Get a specific user by ID"""
    user = await user_service.get_user(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user


@router.get("/users/email/{email}", response_model=User)
async def get_user_by_email(email: str):
    """Get a user by email"""
    user = await user_service.get_user_by_email(email)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user


@router.post("/users", response_model=User)
async def create_user(user_data: UserCreate):
    """Create a new user"""
    # Check if user already exists
    existing_user = await user_service.get_user_by_email(user_data.email)
    if existing_user:
        raise HTTPException(status_code=400, detail="User with this email already exists")

    return await user_service.create_user(user_data)


@router.put("/users/{user_id}", response_model=User)
async def update_user(user_id: UUID, user_data: UserUpdate):
    """Update an existing user"""
    user = await user_service.update_user(user_id, user_data)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    return user


@router.get("/users/{user_id}/progress", response_model=List[UserProgress])
async def get_user_progress(user_id: UUID):
    """Get all progress records for a user"""
    return await user_service.get_user_progress(user_id)


@router.get("/users/{user_id}/progress/{lesson_id}", response_model=UserProgress)
async def get_lesson_progress(user_id: UUID, lesson_id: UUID):
    """Get progress for a specific lesson for a user"""
    progress = await user_service.get_lesson_progress(user_id, lesson_id)
    if not progress:
        raise HTTPException(status_code=404, detail="Progress record not found")
    return progress


@router.post("/users/{user_id}/progress/{lesson_id}", response_model=UserProgress)
async def create_or_update_lesson_progress(
    user_id: UUID,
    lesson_id: UUID,
    progress_data: UserProgressCreate
):
    """Create or update user progress for a lesson"""
    # Ensure the user_id and lesson_id match the path parameters
    progress_data.user_id = user_id
    progress_data.lesson_id = lesson_id

    return await user_service.create_or_update_progress(user_id, lesson_id, progress_data)


@router.put("/users/{user_id}/progress/{lesson_id}", response_model=UserProgress)
async def update_lesson_progress(
    user_id: UUID,
    lesson_id: UUID,
    progress_data: UserProgressUpdate
):
    """Update user progress for a lesson"""
    progress = await user_service.update_progress(user_id, lesson_id, progress_data)
    if not progress:
        raise HTTPException(status_code=404, detail="Progress record not found")
    return progress


@router.get("/users/{user_id}/course-progress")
async def get_course_progress(user_id: UUID):
    """Get overall course progress for a user"""
    progress = await user_service.get_user_course_progress(user_id)
    return progress


@router.get("/users/{user_id}/lesson-completion/{lesson_id}")
async def get_lesson_completion(user_id: UUID, lesson_id: UUID):
    """Get completion percentage for a specific lesson"""
    completion = await user_service.get_user_lesson_completion(user_id, lesson_id)
    return {"completion_percentage": completion}