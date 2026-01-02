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
        if settings.QDRANT_URL == "memory":
            # Use local file-based mode for local development (persistent storage)
            import os
            # Use a persistent location in the project root directory
            project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
            qdrant_path = os.path.join(project_root, "qdrant_local.db")
            self.qdrant_client = QdrantClient(path=qdrant_path)
        else:
            # Use remote Qdrant server
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

        # Check if the client supports the search method
        if hasattr(self.qdrant_client, 'search'):
            # Standard search method exists (remote client or newer in-memory)
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )
        else:
            # Fallback for in-memory client without search method
            # Get all points and calculate similarity manually
            # NOTE: For in-memory client, scroll doesn't return vectors by default
            # We need to retrieve all point IDs first, then get vectors separately
            all_points_result = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                limit=10000,  # Adjust as needed
                with_payload=True,  # Include payloads
                with_vectors=True   # Also include vectors (may not work with all in-memory clients)
            )

            # Handle different return formats depending on client version
            if isinstance(all_points_result, tuple) and len(all_points_result) >= 1:
                all_points = all_points_result[0]  # First element contains the points
            else:
                all_points = all_points_result

            # Calculate cosine similarity manually for in-memory approach
            from sklearn.metrics.pairwise import cosine_similarity
            import numpy as np

            # Extract vectors from points
            vectors = []
            points_with_vectors = []
            for point in all_points:
                # For in-memory client, check if vector is available
                vector_val = getattr(point, 'vector', None)
                if vector_val is not None and hasattr(point, 'payload'):
                    # Vector is available directly
                    vectors.append(vector_val)
                    points_with_vectors.append(point)
                else:
                    # Vector might not be available via scroll in in-memory client
                    # This is the limitation - in-memory client's scroll may not return vectors
                    # In this case, we'll have to handle it differently
                    # Let's check if it's stored in a different attribute
                    if hasattr(point, 'vectors') and hasattr(point, 'payload'):
                        vector_val = point.vectors
                        if isinstance(vector_val, dict) and 'vector' in vector_val:
                            vectors.append(vector_val['vector'])
                            points_with_vectors.append(point)
                        elif isinstance(vector_val, (list, np.ndarray)):
                            vectors.append(vector_val)
                            points_with_vectors.append(point)

            if not vectors:
                # If vectors are still not available, we need to try a different approach
                # This is a known limitation of in-memory Qdrant client with scroll method
                # We'll return empty results in this case
                return []

            # Calculate similarities
            similarities = cosine_similarity([query_embedding], vectors)[0]

            # Create results with similarities as scores
            search_results = []
            for i, point in enumerate(points_with_vectors):
                # Create a mock result object with the same interface
                class MockResult:
                    def __init__(self, point, similarity):
                        self.id = point.id
                        self.score = float(similarity)
                        self.payload = point.payload

                search_results.append(MockResult(point, similarities[i]))

            # Sort by score and take top_k
            search_results.sort(key=lambda x: x.score, reverse=True)
            search_results = search_results[:top_k]

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
        if hasattr(self.qdrant_client, 'search'):
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=search_filter,
                limit=top_k,
                with_payload=True
            )
        else:
            # Handle in-memory client case
            # Get all points and calculate similarity manually
            all_points_result = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                scroll_filter=search_filter if search_filter else None,
                limit=10000  # Adjust as needed
            )

            # Handle different return formats depending on client version
            if isinstance(all_points_result, tuple) and len(all_points_result) >= 1:
                all_points = all_points_result[0]  # First element contains the points
            else:
                all_points = all_points_result

            # Calculate cosine similarity manually for in-memory approach
            from sklearn.metrics.pairwise import cosine_similarity
            import numpy as np

            # Extract vectors from points that match our filter (if any)
            vectors = []
            points_with_vectors = []

            for point in all_points:
                if hasattr(point, 'vector') and hasattr(point, 'payload'):
                    # Apply lesson filter manually if needed
                    if search_filter is None or self._matches_filter(point, search_filter):
                        vectors.append(point.vector)
                        points_with_vectors.append(point)
                elif hasattr(point, 'vectors') and hasattr(point, 'payload'):
                    # Alternative: might be 'vectors' instead of 'vector'
                    vector_val = point.vectors
                    if isinstance(vector_val, dict) and 'vector' in vector_val:
                        vectors.append(vector_val['vector'])
                        if search_filter is None or self._matches_filter(point, search_filter):
                            points_with_vectors.append(point)
                    elif isinstance(vector_val, (list, np.ndarray)):
                        vectors.append(vector_val)
                        if search_filter is None or self._matches_filter(point, search_filter):
                            points_with_vectors.append(point)

            if not vectors:
                return []

            # Calculate similarities
            similarities = cosine_similarity([query_embedding], vectors)[0]

            # Create results with similarities as scores
            search_results = []
            for i, point in enumerate(points_with_vectors):
                # Create a mock result object with the same interface
                class MockResult:
                    def __init__(self, point, similarity):
                        self.id = point.id
                        self.score = float(similarity)
                        self.payload = point.payload

                search_results.append(MockResult(point, similarities[i]))

            # Sort by score and take top_k
            search_results.sort(key=lambda x: x.score, reverse=True)
            search_results = search_results[:top_k]

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

    def _matches_filter(self, point, search_filter):
        """Helper method to manually apply filters for in-memory client"""
        if not search_filter or not search_filter.must:
            return True

        for condition in search_filter.must:
            if hasattr(condition, 'key') and condition.key == "payload.lesson_id":
                if hasattr(condition, 'match'):
                    if hasattr(condition.match, 'any'):
                        # MatchAny case
                        lesson_id = point.payload.get("lesson_id", "")
                        if lesson_id in condition.match.any:
                            return True
                    elif hasattr(condition.match, 'value'):
                        # MatchValue case
                        lesson_id = point.payload.get("lesson_id", "")
                        if lesson_id == condition.match.value:
                            return True

        return False


# Global service instance
rag_service = RAGService()