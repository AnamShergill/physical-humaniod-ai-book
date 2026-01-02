from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from sentence_transformers import SentenceTransformer
import numpy as np

# Create in-memory client
client = QdrantClient(":memory:")

# Create a collection
collection_name = "test_collection"
client.create_collection(
    collection_name=collection_name,
    vectors_config=models.VectorParams(size=384, distance=models.Distance.COSINE)  # all-MiniLM-L6-v2 produces 384-dim vectors
)

# Add some test data
test_vectors = [
    [float(i) for i in range(384)],  # Simple test vector
    [float(i + 0.5) for i in range(384)]  # Another test vector
]

client.upsert(
    collection_name=collection_name,
    points=[
        PointStruct(
            id=1,
            vector=test_vectors[0],
            payload={"content": "This is about Physical AI", "lesson_id": "test-lesson-1"}
        ),
        PointStruct(
            id=2,
            vector=test_vectors[1],
            payload={"content": "This is about robotics", "lesson_id": "test-lesson-2"}
        )
    ]
)

print("Added test data to collection")

# Test scroll method
print("\nTesting scroll method:")
result = client.scroll(collection_name=collection_name, limit=10)
print("Scroll result type:", type(result))
print("Scroll result length:", len(result) if isinstance(result, (list, tuple)) else "N/A")
print("Full scroll result:", result)

# Check if it's a tuple (which is common)
if isinstance(result, tuple):
    points = result[0]
    next_offset = result[1] if len(result) > 1 else None
    print(f"Points from tuple[0]: {len(points) if points else 0} items")
    print(f"Next offset: {next_offset}")
    for i, point in enumerate(points):
        print(f"Point {i}: ID={point.id}, Payload={point.payload}, Vector type={type(getattr(point, 'vector', 'NO_VECTOR_ATTR'))}")
else:
    print("Result is not a tuple")
    for i, point in enumerate(result):
        print(f"Point {i}: ID={point.id}, Payload={point.payload}")

# Test if search method exists (should be False based on previous test)
print(f"\nHas search method: {hasattr(client, 'search')}")

# If search doesn't exist, try manual similarity search
if not hasattr(client, 'search'):
    print("\nTesting manual similarity search...")
    encoder = SentenceTransformer('all-MiniLM-L6-v2')
    query_embedding = encoder.encode("What is Physical AI?").tolist()

    # Get all points
    all_points_result = client.scroll(collection_name=collection_name, limit=10)
    all_points = all_points_result[0] if isinstance(all_points_result, tuple) else all_points_result

    print(f"Retrieved {len(all_points)} points")

    # Calculate similarity manually
    from sklearn.metrics.pairwise import cosine_similarity

    vectors = []
    points_with_vectors = []
    for point in all_points:
        print(f"Point ID: {point.id}, Vector attr: {hasattr(point, 'vector')}, Payload: {point.payload}")
        if hasattr(point, 'vector'):
            vectors.append(point.vector)
            points_with_vectors.append(point)

    if vectors:
        similarities = cosine_similarity([query_embedding], vectors)[0]
        print(f"Similarities: {similarities}")

        # Find the best match
        best_idx = np.argmax(similarities)
        best_match = points_with_vectors[best_idx]
        print(f"Best match: ID={best_match.id}, Similarity={similarities[best_idx]}, Content='{best_match.payload.get('content')}'")
    else:
        print("No vectors found!")