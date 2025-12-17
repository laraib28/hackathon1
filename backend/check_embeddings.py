"""
Quick script to check if embeddings exist in Qdrant collection
"""

import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

COLLECTION_NAME = "humanoid_robotics_docs"

try:
    qdrant = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    collection_info = qdrant.get_collection(COLLECTION_NAME)

    print(f"✅ Collection: {COLLECTION_NAME}")
    print(f"📊 Total vectors: {collection_info.points_count}")
    print(f"📐 Vector size: {collection_info.config.params.vectors.size}")
    print(f"📍 Distance: {collection_info.config.params.vectors.distance}")
    print(f"🟢 Status: {collection_info.status}")

    if collection_info.points_count == 0:
        print("\n⚠️  No embeddings found! Run populate_embeddings.py to add documents.")
    else:
        print(f"\n✅ Collection has {collection_info.points_count} embeddings!")

except Exception as e:
    print(f"❌ Error: {e}")
    print("Collection may not exist yet. Run populate_embeddings.py to create it.")
