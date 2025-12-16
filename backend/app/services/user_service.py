from typing import List, Optional
from uuid import UUID, uuid4
from datetime import datetime
import logging

from app.models.user import User, UserCreate, UserUpdate, UserProgress, UserProgressCreate, UserProgressUpdate
from app.config import settings

logger = logging.getLogger(__name__)


class UserService:
    def __init__(self):
        # In a real implementation, this would connect to Neon database
        self.users = {}
        self.user_progress = {}

    async def get_user(self, user_id: UUID) -> Optional[User]:
        """Get a user by ID"""
        return self.users.get(user_id)

    async def get_user_by_email(self, email: str) -> Optional[User]:
        """Get a user by email"""
        for user in self.users.values():
            if user.email == email:
                return user
        return None

    async def create_user(self, user_data: UserCreate) -> User:
        """Create a new user"""
        user_id = uuid4()
        user = User(
            id=user_id,
            **user_data.dict(exclude={'password'}),  # Password would be hashed in real app
            created_at=datetime.utcnow(),
            updated_at=datetime.utcnow()
        )
        self.users[user_id] = user
        return user

    async def update_user(self, user_id: UUID, user_data: UserUpdate) -> Optional[User]:
        """Update an existing user"""
        if user_id not in self.users:
            return None

        user = self.users[user_id]
        update_data = user_data.dict(exclude_unset=True)
        for field, value in update_data.items():
            setattr(user, field, value)
        user.updated_at = datetime.utcnow()
        self.users[user_id] = user
        return user

    async def get_user_progress(self, user_id: UUID) -> List[UserProgress]:
        """Get all progress records for a user"""
        user_progress = []
        for progress in self.user_progress.values():
            if progress.user_id == user_id:
                user_progress.append(progress)
        return user_progress

    async def get_lesson_progress(self, user_id: UUID, lesson_id: UUID) -> Optional[UserProgress]:
        """Get progress for a specific lesson for a user"""
        for progress in self.user_progress.values():
            if progress.user_id == user_id and progress.lesson_id == lesson_id:
                return progress
        return None

    async def create_or_update_progress(self, user_id: UUID, lesson_id: UUID, progress_data: UserProgressCreate) -> UserProgress:
        """Create or update user progress for a lesson"""
        # Check if progress already exists
        existing_progress = await self.get_lesson_progress(user_id, lesson_id)

        if existing_progress:
            # Update existing progress
            progress_id = existing_progress.id
            progress = UserProgress(
                id=progress_id,
                **progress_data.dict(),
                updated_at=datetime.utcnow()
            )
            self.user_progress[progress_id] = progress
        else:
            # Create new progress
            progress_id = uuid4()
            progress = UserProgress(
                id=progress_id,
                **progress_data.dict(),
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow()
            )
            self.user_progress[progress_id] = progress

        return progress

    async def update_progress(self, user_id: UUID, lesson_id: UUID, progress_data: UserProgressUpdate) -> Optional[UserProgress]:
        """Update user progress for a lesson"""
        existing_progress = await self.get_lesson_progress(user_id, lesson_id)
        if not existing_progress:
            return None

        progress_id = existing_progress.id
        update_data = progress_data.dict(exclude_unset=True)

        # Update the progress object
        for field, value in update_data.items():
            setattr(existing_progress, field, value)
        existing_progress.updated_at = datetime.utcnow()

        self.user_progress[progress_id] = existing_progress
        return existing_progress

    async def get_user_lesson_completion(self, user_id: UUID, lesson_id: UUID) -> int:
        """Get completion percentage for a specific lesson"""
        progress = await self.get_lesson_progress(user_id, lesson_id)
        if progress:
            return progress.completion_percentage
        return 0

    async def get_user_course_progress(self, user_id: UUID) -> Dict[str, int]:
        """Get overall course progress for a user"""
        user_progress_list = await self.get_user_progress(user_id)

        if not user_progress_list:
            return {
                "total_lessons": 0,
                "completed_lessons": 0,
                "completion_percentage": 0
            }

        total_lessons = len(user_progress_list)
        completed_lessons = sum(1 for progress in user_progress_list if progress.completion_status == "completed")
        completion_percentage = (completed_lessons / total_lessons) * 100 if total_lessons > 0 else 0

        return {
            "total_lessons": total_lessons,
            "completed_lessons": completed_lessons,
            "completion_percentage": completion_percentage
        }


# Global service instance
user_service = UserService()