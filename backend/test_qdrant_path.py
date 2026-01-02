from app.services.rag_service import rag_service
import os

# Check if the Qdrant database file exists
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
qdrant_db_path = os.path.join(project_root, "qdrant_local.db")

print(f"Project root: {project_root}")
print(f"Qdrant DB path: {qdrant_db_path}")
print(f"Qdrant DB exists: {os.path.exists(qdrant_db_path)}")

if os.path.exists(qdrant_db_path):
    print(f"Qdrant DB size: {os.path.getsize(qdrant_db_path)} bytes")

# Check the collection
try:
    collection_info = rag_service.qdrant_client.get_collection(rag_service.collection_name)
    print(f'Collection exists with {collection_info.points_count} vectors')
except Exception as e:
    print(f'Collection error: {str(e)}')

# List files in the project root to see what's there
print(f"\nFiles in project root ({project_root}):")
for item in os.listdir(project_root):
    item_path = os.path.join(project_root, item)
    if os.path.isdir(item_path):
        print(f"  [DIR]  {item}")
    else:
        print(f"  [FILE] {item} ({os.path.getsize(item_path)} bytes)")