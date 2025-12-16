from fastapi import APIRouter, HTTPException, Query
from typing import List, Dict, Any, Optional
from uuid import UUID

from app.services.rag_service import rag_service
from app.services.lesson_service import lesson_service

router = APIRouter()


@router.post("/rag/search")
async def search_content(
    query: str = Query(..., min_length=3, max_length=500),
    top_k: int = Query(5, ge=1, le=20),
    lesson_ids: List[UUID] = Query(None)
):
    """Search for relevant content using RAG"""
    if lesson_ids:
        results = await rag_service.get_relevant_context(query, lesson_ids, top_k)
    else:
        results = await rag_service.search_similar_content(query, top_k)

    return {
        "query": query,
        "results": results,
        "count": len(results)
    }


@router.get("/rag/lesson/{lesson_id}/context")
async def get_lesson_context(lesson_id: UUID):
    """Get all context chunks for a specific lesson"""
    results = await rag_service.get_lesson_context(lesson_id)
    return {
        "lesson_id": lesson_id,
        "context_chunks": results,
        "count": len(results)
    }


@router.post("/rag/index-lesson/{lesson_id}")
async def index_lesson_content(lesson_id: UUID):
    """Index a lesson's content in the vector database"""
    # Get the lesson
    lesson = await lesson_service.get_lesson(lesson_id)
    if not lesson:
        raise HTTPException(status_code=404, detail="Lesson not found")

    # Get the lesson chunks
    chunks = await lesson_service.get_lesson_chunks(lesson_id)
    if not chunks:
        # If no chunks exist, create them first
        chunks = await lesson_service.chunk_lesson_content(lesson_id)

    # Index the chunks in Qdrant
    await rag_service.index_lesson_chunks(chunks)

    return {
        "message": f"Indexed {len(chunks)} chunks for lesson {lesson_id}",
        "lesson_title": lesson.title
    }


@router.delete("/rag/lesson/{lesson_id}")
async def delete_lesson_from_index(lesson_id: UUID):
    """Remove a lesson's content from the vector database"""
    await rag_service.delete_lesson_embeddings(lesson_id)
    return {
        "message": f"Removed embeddings for lesson {lesson_id} from vector database"
    }


@router.post("/rag/answer-question")
async def answer_question(
    question: str = Query(..., min_length=5, max_length=1000),
    lesson_ids: List[UUID] = Query(None),
    top_k: int = Query(3, ge=1, le=10)
):
    """Answer a question using RAG with lesson content"""
    # Get relevant context
    context_results = await rag_service.get_relevant_context(
        query=question,
        lesson_ids=lesson_ids,
        top_k=top_k
    )

    # In a real implementation, you would use an LLM to generate the answer
    # based on the retrieved context. For now, we'll return the context.

    # Build context string
    context_texts = [result["content"] for result in context_results]
    context_string = "\n\n".join(context_texts)

    # Generate a simple answer based on the context
    # (In practice, this would be done by an LLM)
    answer = f"Based on the provided context: {context_string[:500]}..." if context_string else "No relevant context found."

    return {
        "question": question,
        "answer": answer,
        "context_used": context_texts,
        "relevance_scores": [result["relevance_score"] for result in context_results]
    }