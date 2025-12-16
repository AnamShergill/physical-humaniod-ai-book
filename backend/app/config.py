from pydantic_settings import BaseSettings
from typing import List, Optional


class Settings(BaseSettings):
    # API Settings
    API_V1_STR: str = "/api/v1"
    PROJECT_NAME: str = "Physical AI & Humanoid Robotics Textbook"
    VERSION: str = "1.0.0"

    # Database Settings
    DATABASE_URL: str
    QDRANT_URL: str
    QDRANT_API_KEY: Optional[str] = None

    # Neon Database Settings
    NEON_DATABASE_URL: str

    # CORS Settings
    ALLOWED_ORIGINS: List[str] = ["*"]  # Should be configured properly in production

    # Application Settings
    DEBUG: bool = False
    MAX_CONTENT_LENGTH: int = 10 * 1024 * 1024  # 10MB

    # Lesson Settings
    LESSON_CHUNK_SIZE: int = 1000  # Characters per chunk
    LESSON_CHUNK_OVERLAP: int = 100  # Overlap between chunks

    class Config:
        env_file = ".env"


settings = Settings()