from pydantic import BaseModel
from typing import Optional, Dict
from datetime import datetime
from uuid import UUID


class UserBase(BaseModel):
    email: str
    name: Optional[str] = None
    language_preference: str = "en"
    preferences: Optional[Dict] = {}


class UserCreate(UserBase):
    password: str  # In real app, would be hashed


class UserUpdate(BaseModel):
    name: Optional[str] = None
    language_preference: Optional[str] = None
    preferences: Optional[Dict] = None


class User(UserBase):
    id: UUID
    created_at: datetime
    updated_at: datetime
    last_active: Optional[datetime] = None

    class Config:
        from_attributes = True


class UserProgressBase(BaseModel):
    user_id: UUID
    lesson_id: UUID
    completion_status: str  # 'not_started', 'in_progress', 'completed'
    completion_percentage: int
    time_spent: int  # in seconds
    notes: Optional[str] = None


class UserProgressCreate(UserProgressBase):
    pass


class UserProgressUpdate(BaseModel):
    completion_status: Optional[str] = None
    completion_percentage: Optional[int] = None
    time_spent: Optional[int] = None
    notes: Optional[str] = None


class UserProgress(UserProgressBase):
    id: UUID
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True