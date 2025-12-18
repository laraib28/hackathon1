"""
Recreate Qdrant collection with correct dimensions
"""

import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

load_dotenv()

COLLECTION_NAME = "humanoid_robotics_docs"
VECTOR_SIZE = 384  # For SentenceTransformers

def recreate_collection():
    """Delete and recreate collection with correct dimensions"""

    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    try:
        # Check if collection exists
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        print(f"📊 Existing collection:")
        print(f"   Vectors: {collection_info.points_count}")
        print(f"   Vector size: {collection_info.config.params.vectors.size}")

        # Delete collection
        print(f"\n🗑️  Deleting collection '{COLLECTION_NAME}'...")
        qdrant.delete_collection(COLLECTION_NAME)
        print("✅ Collection deleted")

    except Exception as e:
        print(f"Collection doesn't exist or error: {e}")

    # Create new collection
    print(f"\n📦 Creating collection with {VECTOR_SIZE} dimensions...")
    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(
            size=VECTOR_SIZE,
            distance=models.Distance.COSINE
        )
    )

    # Verify
    collection_info = qdrant.get_collection(COLLECTION_NAME)
    print(f"\n✅ Collection created successfully!")
    print(f"   Name: {COLLECTION_NAME}")
    print(f"   Vector size: {collection_info.config.params.vectors.size}")
    print(f"   Distance: {collection_info.config.params.vectors.distance}")
    print(f"   Status: {collection_info.status}")

if __name__ == "__main__":
    try:
        recreate_collection()
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
