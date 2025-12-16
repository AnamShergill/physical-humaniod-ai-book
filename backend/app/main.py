from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api import lesson_routes, rag_routes, user_routes
from app.config import settings

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="API for the Physical AI & Humanoid Robotics textbook",
    version="1.0.0",
    openapi_url="/api/v1/openapi.json",
    docs_url="/api/v1/docs",
    redoc_url="/api/v1/redoc"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(lesson_routes.router, prefix="/api/v1", tags=["lessons"])
app.include_router(rag_routes.router, prefix="/api/v1", tags=["rag"])
app.include_router(user_routes.router, prefix="/api/v1", tags=["users"])

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "textbook-api"}

@app.get("/")
async def root():
    return {"message": "Physical AI & Humanoid Robotics Textbook API"}