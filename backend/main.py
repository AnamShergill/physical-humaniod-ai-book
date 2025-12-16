"""
Main entry point for the Physical AI & Humanoid Robotics Textbook backend
"""
import uvicorn
import asyncio
import logging
from app.main import app
from app.database import init_db, test_connection

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """
    Main function to start the FastAPI application
    """
    logger.info("Initializing database...")
    init_db()

    logger.info("Testing database connection...")
    if not test_connection():
        logger.error("Database connection failed. Exiting.")
        return

    logger.info("Starting the Physical AI & Humanoid Robotics Textbook API...")

    # Run the application
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,  # Set to False in production
        log_level="info"
    )


if __name__ == "__main__":
    main()