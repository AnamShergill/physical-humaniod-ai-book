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


@router.post("/chatbot/query")
async def query_chatbot(
    query: str = Query(..., min_length=5, max_length=1000),
    context: Optional[str] = Query(None)
):
    """Chatbot endpoint that answers questions based on textbook content"""
    try:
        print(f"Chatbot query received: {query}")
        print(f"Context received: {context}")

        # Process the context to identify specific lessons/chapters if provided
        lesson_ids = None
        if context:
            # In a real implementation, we would parse the context to identify specific lesson IDs
            # For now, we'll just log the context for debugging
            print(f"Context received: {context}")
            # You could implement logic here to map context like "foundations-of-physical-ai/lesson-1.1"
            # to specific lesson IDs in the database

        # Search for relevant content based on context
        if lesson_ids:
            # If specific lesson IDs are identified, search within those lessons only
            print("Searching within specific lessons...")
            context_results = await rag_service.get_relevant_context(query, lesson_ids, top_k=5)
        else:
            # Otherwise, search across all content
            print("Searching across all content...")
            context_results = await rag_service.search_similar_content(query, top_k=5)

        print(f"Found {len(context_results)} results from RAG search")

        # Build context string
        context_texts = [result["content"] for result in context_results]
        sources = [result.get("payload", {}).get("source", "Unknown source") for result in context_results]
        relevance_scores = [result.get("relevance_score", result.get("score", 0.0)) for result in context_results]
        context_string = "\n\n".join(context_texts[:3])  # Limit to first 3 chunks

        # Generate a simple answer based on the context
        if context_string:
            answer = f"Based on the textbook content: {context_string[:800]}..."
        else:
            answer = "I couldn't find relevant information in the textbook to answer your question. The textbook may not have been properly indexed yet."

        print(f"Returning answer: {answer[:100]}...")

        return {
            "response": answer,
            "sources": sources,
            "confidence": max(relevance_scores) if relevance_scores else 0.5
        }
    except Exception as e:
        print(f"Error in chatbot query: {str(e)}")
        import traceback
        print(f"Traceback: {traceback.format_exc()}")

        return {
            "response": f"Sorry, I encountered an error processing your question: {str(e)}",
            "sources": [],
            "confidence": 0.0
        }