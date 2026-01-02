from qdrant_client import QdrantClient

# Create in-memory client
client = QdrantClient(":memory:")

print("Client type:", type(client))
print("Has search method:", hasattr(client, 'search'))
print("Has scroll method:", hasattr(client, 'scroll'))

# Try to list all attributes that might be related to search
attrs = [attr for attr in dir(client) if 'search' in attr.lower() or 'similarity' in attr.lower() or 'vector' in attr.lower()]
print("Potential search-related attributes:", attrs)