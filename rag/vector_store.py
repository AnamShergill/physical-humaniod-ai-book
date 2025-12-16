"""
Vector store integration for the RAG system using Qdrant
"""
from typing import List, Dict, Any, Optional
from uuid import UUID
import logging
from sentence_transformers import SentenceTransformer

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct

from rag.lesson_chunker import LessonChunk

logger = logging.getLogger(__name__)


class VectorStore:
    """
    Vector store interface for the RAG system using Qdrant
    """
    def __init__(self, url: str, api_key: Optional[str] = None, collection_name: str = "lesson_embeddings"):
        self.client = QdrantClient(url=url, api_key=api_key, prefer_grpc=True)
        self.collection_name = collection_name
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')

        # Create collection if it doesn't exist
        self._create_collection()

    def _create_collection(self):
        """
        Create Qdrant collection for lesson embeddings if it doesn't exist
        """
        try:
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.encoder.get_sentence_embedding_dimension(),  # Usually 384 for all-MiniLM-L6-v2
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created Qdrant collection: {self.collection_name}")

    def add_lesson_chunks(self, chunks: List[LessonChunk]) -> bool:
        """
        Add lesson chunks to the vector store
        """
        points = []
        for chunk in chunks:
            # Generate embedding for the chunk content
            embedding = self.encoder.encode(chunk.content).tolist()

            # Create Qdrant point
            point = PointStruct(
                id=chunk.id,
                vector=embedding,
                payload={
                    "lesson_id": chunk.lesson_id,
                    "chunk_order": chunk.chunk_order,
                    "content": chunk.content,
                    "token_count": chunk.token_count,
                    "metadata": chunk.metadata
                }
            )
            points.append(point)

        if points:
            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Added {len(points)} chunks to vector store")
            return True

        return False

    def search(self, query: str, top_k: int = 5, lesson_ids: List[str] = None) -> List[Dict[str, Any]]:
        """
        Search for relevant chunks based on query
        """
        # Generate embedding for the query
        query_embedding = self.encoder.encode(query).tolist()

        # Create filter if lesson IDs are specified
        search_filter = None
        if lesson_ids:
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="payload.lesson_id",
                        match=models.MatchAny(any=lesson_ids)
                    )
                ]
            )

        # Search in Qdrant
        search_results = self.client.search(
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
                "metadata": hit.payload.get("metadata", {}),
                "relevance_score": hit.score
            })

        return results

    def get_lesson_chunks(self, lesson_id: str) -> List[Dict[str, Any]]:
        """
        Get all chunks for a specific lesson
        """
        filter_condition = models.Filter(
            must=[
                models.FieldCondition(
                    key="payload.lesson_id",
                    match=models.MatchValue(value=lesson_id)
                )
            ]
        )

        search_results = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=filter_condition,
            limit=100  # Adjust as needed
        )

        results = []
        for point in search_results[0]:  # First element contains the points
            results.append({
                "id": point.id,
                "payload": point.payload,
                "content": point.payload.get("content", ""),
                "chunk_order": point.payload.get("chunk_order", 0)
            })

        # Sort by chunk order
        results.sort(key=lambda x: x["chunk_order"])
        return results

    def delete_lesson_chunks(self, lesson_id: str) -> bool:
        """
        Delete all chunks for a specific lesson
        """
        filter_condition = models.Filter(
            must=[
                models.FieldCondition(
                    key="payload.lesson_id",
                    match=models.MatchValue(value=lesson_id)
                )
            ]
        )

        # Get all points to delete
        points_to_delete = []
        scroll_result = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=filter_condition,
            limit=1000  # Adjust as needed
        )

        for point in scroll_result[0]:  # First element contains the points
            points_to_delete.append(point.id)

        if points_to_delete:
            # Delete points
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=points_to_delete
                )
            )
            logger.info(f"Deleted {len(points_to_delete)} chunks for lesson {lesson_id}")
            return True

        return False

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection
        """
        collection_info = self.client.get_collection(self.collection_name)
        return {
            "name": collection_info.config.params.vectors_count,
            "vector_size": collection_info.config.params.vector_size,
            "distance": collection_info.config.params.distance
        }


# Global vector store instance (will be initialized with proper config)
# vector_store = VectorStore(url="your-qdrant-url", api_key="your-api-key")