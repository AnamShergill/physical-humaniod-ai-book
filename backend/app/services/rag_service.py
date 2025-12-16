from typing import List, Dict, Any
from uuid import UUID
import logging
from sentence_transformers import SentenceTransformer
import numpy as np

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct

from app.models.lesson import LessonChunk
from app.config import settings

logger = logging.getLogger(__name__)


class RAGService:
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True
        )

        # Initialize sentence transformer model
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')

        # Collection name for lesson embeddings
        self.collection_name = "lesson_embeddings"

        # Create collection if it doesn't exist
        self._create_collection()

    def _create_collection(self):
        """Create Qdrant collection for lesson embeddings"""
        try:
            self.qdrant_client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.encoder.get_sentence_embedding_dimension(),  # Usually 384 for all-MiniLM-L6-v2
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created Qdrant collection: {self.collection_name}")

    async def index_lesson_chunks(self, lesson_chunks: List[LessonChunk]):
        """Index lesson chunks in Qdrant"""
        points = []

        for chunk in lesson_chunks:
            # Generate embedding for the chunk content
            embedding = self.encoder.encode(chunk.content_chunk).tolist()

            # Create Qdrant point
            point = PointStruct(
                id=chunk.embedding_id,
                vector=embedding,
                payload={
                    "lesson_id": str(chunk.lesson_id),
                    "chunk_order": chunk.chunk_order,
                    "content": chunk.content_chunk,
                    "created_at": chunk.created_at.isoformat()
                }
            )
            points.append(point)

        if points:
            # Upload points to Qdrant
            self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Indexed {len(points)} lesson chunks in Qdrant")

    async def search_similar_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """Search for similar content using vector similarity"""
        # Generate embedding for the query
        query_embedding = self.encoder.encode(query).tolist()

        # Search in Qdrant
        search_results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k,
            with_payload=True
        )

        results = []
        for hit in search_results:
            results.append({
                "id": hit.id,
                "score": hit.score,
                "payload": hit.payload,
                "content": hit.payload.get("content", "")
            })

        return results

    async def get_lesson_context(self, lesson_id: UUID) -> List[Dict[str, Any]]:
        """Get all chunks for a specific lesson"""
        # Search for all chunks belonging to the lesson
        filter_condition = models.Filter(
            must=[
                models.FieldCondition(
                    key="payload.lesson_id",
                    match=models.MatchValue(value=str(lesson_id))
                )
            ]
        )

        search_results = self.qdrant_client.scroll(
            collection_name=self.collection_name,
            scroll_filter=filter_condition,
            limit=100  # Adjust as needed
        )

        results = []
        for point in search_results[0]:  # First element contains the points
            results.append({
                "id": point.id,
                "payload": point.payload,
                "content": point.payload.get("content", "")
            })

        # Sort by chunk order
        results.sort(key=lambda x: x["payload"].get("chunk_order", 0))
        return results

    async def delete_lesson_embeddings(self, lesson_id: UUID):
        """Delete all embeddings for a lesson"""
        filter_condition = models.Filter(
            must=[
                models.FieldCondition(
                    key="payload.lesson_id",
                    match=models.MatchValue(value=str(lesson_id))
                )
            ]
        )

        # Get all points to delete
        points_to_delete = []
        scroll_result = self.qdrant_client.scroll(
            collection_name=self.collection_name,
            scroll_filter=filter_condition,
            limit=1000  # Adjust as needed
        )

        for point in scroll_result[0]:  # First element contains the points
            points_to_delete.append(point.id)

        if points_to_delete:
            # Delete points
            self.qdrant_client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=points_to_delete
                )
            )
            logger.info(f"Deleted {len(points_to_delete)} embeddings for lesson {lesson_id}")

    async def get_relevant_context(self, query: str, lesson_ids: List[UUID] = None, top_k: int = 5) -> List[Dict[str, Any]]:
        """Get relevant context for a query, optionally limited to specific lessons"""
        # Generate embedding for the query
        query_embedding = self.encoder.encode(query).tolist()

        # Create filter if lesson IDs are specified
        search_filter = None
        if lesson_ids:
            lesson_id_values = [str(lesson_id) for lesson_id in lesson_ids]
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="payload.lesson_id",
                        match=models.MatchAny(any=lesson_id_values)
                    )
                ]
            )

        # Search in Qdrant
        search_results = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=search_filter,
            limit=top_k,
            with_payload=True
        )

        results = []
        for hit in search_results:
            results.append({
                "id": hit.id,
                "score": hit.score,
                "lesson_id": hit.payload.get("lesson_id"),
                "chunk_order": hit.payload.get("chunk_order"),
                "content": hit.payload.get("content", ""),
                "relevance_score": hit.score
            })

        return results


# Global service instance
rag_service = RAGService()