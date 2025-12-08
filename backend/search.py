"""
Semantic Search Module
Retrieve data from Qdrant using Sentence Transformers (local embeddings - 100% FREE!)
"""

from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from typing import List, Dict, Optional
import json
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
COLLECTION_NAME = "humanoid_robotics_docs"

# Load model (runs locally - no API needed!)
print("📥 Loading embedding model...")
model = SentenceTransformer('all-MiniLM-L6-v2')
print("✅ Model ready!")

qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

def generate_query_embedding(query: str) -> List[float]:
    """
    Generate embedding for search query using Sentence Transformers (local)
    """
    embedding = model.encode(query, convert_to_numpy=True)
    return embedding.tolist()

def search(
    query: str,
    limit: int = 5,
    score_threshold: float = 0.7
) -> List[Dict]:
    """
    Perform semantic search

    Args:
        query: Search query text
        limit: Number of results to return
        score_threshold: Minimum similarity score (0-1)

    Returns:
        List of search results with content and metadata
    """
    try:
        # Generate query embedding
        print(f"🔍 Searching for: '{query}'")
        query_vector = generate_query_embedding(query)

        # Search in Qdrant
        results = qdrant.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=limit,
            score_threshold=score_threshold
        )

        # Format results
        formatted_results = []
        for idx, result in enumerate(results, 1):
            formatted_results.append({
                "rank": idx,
                "score": round(result.score, 4),
                "url": result.payload.get("url", ""),
                "content": result.payload.get("content", ""),
                "chunk_index": result.payload.get("chunk_index", 0),
                "total_chunks": result.payload.get("total_chunks", 0),
                "source": result.payload.get("source", "")
            })

        return formatted_results

    except Exception as e:
        print(f"❌ Search error: {e}")
        return []

def search_with_context(
    query: str,
    limit: int = 5,
    context_chunks: int = 1
) -> List[Dict]:
    """
    Search with surrounding context chunks
    Retrieves adjacent chunks for better context
    """
    results = search(query, limit)

    # For each result, could fetch adjacent chunks
    # This would require additional Qdrant queries
    # Simplified version returns original results
    return results

def get_collection_stats() -> Dict:
    """
    Get statistics about the collection
    """
    try:
        collection_info = qdrant.get_collection(COLLECTION_NAME)

        return {
            "collection_name": COLLECTION_NAME,
            "vectors_count": collection_info.points_count,
            "vector_size": collection_info.config.params.vectors.size,
            "distance": collection_info.config.params.vectors.distance,
            "status": collection_info.status
        }
    except Exception as e:
        return {"error": str(e)}

def interactive_search():
    """
    Interactive search mode for testing
    """
    print("=" * 60)
    print("🔍 Semantic Search - Interactive Mode (100% FREE)")
    print("=" * 60)

    # Show stats
    stats = get_collection_stats()
    print(f"\n📊 Collection Stats:")
    print(f"  Name: {stats.get('collection_name')}")
    print(f"  Vectors: {stats.get('vectors_count', 0)}")
    print(f"  Status: {stats.get('status', 'unknown')}")
    print(f"  Mode: LOCAL (No API needed!)")
    print("\n" + "=" * 60)

    while True:
        print("\nEnter your search query (or 'quit' to exit):")
        query = input(">>> ").strip()

        if query.lower() in ['quit', 'exit', 'q']:
            print("\n👋 Goodbye!")
            break

        if not query:
            continue

        # Perform search
        results = search(query, limit=5, score_threshold=0.5)

        if not results:
            print("\n❌ No results found. Try a different query.")
            continue

        # Display results
        print(f"\n✅ Found {len(results)} results:\n")

        for result in results:
            print(f"[{result['rank']}] Score: {result['score']:.2%}")
            print(f"    URL: {result['url']}")
            print(f"    Content: {result['content'][:200]}...")
            if result['total_chunks'] > 1:
                print(f"    (Chunk {result['chunk_index'] + 1}/{result['total_chunks']})")
            print()

def main():
    """
    Main function - run interactive search
    """
    try:
        interactive_search()
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == "__main__":
    main()
