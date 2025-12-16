# Physical AI & Humanoid Robotics Textbook - Backend

This is the backend service for the Physical AI & Humanoid Robotics Textbook, built with FastAPI and designed to work with Neon PostgreSQL and Qdrant vector database.

## Architecture

The backend follows a modular architecture:

- **API Routes**: FastAPI endpoints in `app/api/`
- **Services**: Business logic in `app/services/`
- **Models**: Pydantic models in `app/models/`
- **Database**: SQLAlchemy models for Neon PostgreSQL in `app/models/db_models.py`
- **RAG System**: Vector search capabilities with Qdrant
- **Agents**: Reusable AI skills in the `/agents` directory

## Setup

### Prerequisites

- Python 3.8+
- Neon PostgreSQL database
- Qdrant vector database

### Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables in `.env`:
```env
DATABASE_URL=postgresql://username:password@localhost:5432/textbook
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=your-api-key
NEON_DATABASE_URL=postgresql://username:password@ep-...
ALLOWED_ORIGINS=["http://localhost:3000", "http://localhost:3001"]
DEBUG=True
```

### Running the Application

```bash
python main.py
```

Or with uvicorn directly:
```bash
uvicorn app.main:app --reload --port 8000
```

## Importing Content

To import the existing textbook content into the database:

```bash
python import_content.py
```

This will:
1. Parse the Markdown files in the website/docs/chapters directory
2. Create Chapter and Lesson records in Neon PostgreSQL
3. Chunk lesson content and index it in Qdrant

## API Endpoints

### Lessons
- `GET /api/v1/lessons` - Get all lessons
- `GET /api/v1/lessons/{lesson_id}` - Get specific lesson
- `POST /api/v1/lessons` - Create lesson
- `PUT /api/v1/lessons/{lesson_id}` - Update lesson
- `DELETE /api/v1/lessons/{lesson_id}` - Delete lesson

### RAG (Retrieval Augmented Generation)
- `POST /api/v1/rag/search` - Search for relevant content
- `GET /api/v1/rag/lesson/{lesson_id}/context` - Get lesson context
- `POST /api/v1/rag/index-lesson/{lesson_id}` - Index lesson in vector store
- `POST /api/v1/rag/answer-question` - Answer question using RAG

### Users
- `GET /api/v1/users/{user_id}` - Get user
- `POST /api/v1/users` - Create user
- `GET /api/v1/users/{user_id}/progress` - Get user progress
- `POST /api/v1/users/{user_id}/progress/{lesson_id}` - Update progress

## Environment Variables

- `DATABASE_URL`: Neon PostgreSQL connection string
- `QDRANT_URL`: Qdrant instance URL
- `QDRANT_API_KEY`: Qdrant API key (if required)
- `NEON_DATABASE_URL`: Neon database URL
- `ALLOWED_ORIGINS`: Comma-separated list of allowed origins
- `DEBUG`: Enable debug mode

## Database Models

The backend uses SQLAlchemy models stored in Neon PostgreSQL:

- **Chapter**: Represents textbook chapters
- **Lesson**: Individual lessons within chapters
- **LessonChunk**: Chunked content for vector search
- **User**: User accounts and preferences
- **UserProgress**: Track user progress through lessons
- **Translation**: Multilingual content support

## RAG System

The RAG (Retrieval Augmented Generation) system uses:
- Qdrant as the vector database
- Sentence Transformers for embedding generation
- Lesson-level chunking for precise retrieval

Content is chunked at the lesson level and stored in Qdrant for semantic search capabilities.

## Agent Skills

Reusable AI agent skills are available in the `/agents` directory:
- Lesson generation
- Content summarization
- Quiz generation
- Translation services

## Deployment

The backend is designed for deployment on Railway with:
- Neon PostgreSQL for structured data
- Qdrant for vector storage
- FastAPI for efficient API handling