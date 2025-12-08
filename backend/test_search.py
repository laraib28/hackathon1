"""
Test Search Functionality
Sample queries to test retrieval
"""

import search as search_module
import json

def test_search():
    """
    Test search with sample queries
    """
    print("=" * 60)
    print("🧪 Testing Semantic Search")
    print("=" * 60)

    # Show collection stats
    print("\n📊 Collection Statistics:")
    stats = search_module.get_collection_stats()
    print(json.dumps(stats, indent=2))

    # Sample queries
    test_queries = [
        "What is ROS 2?",
        "How do I simulate sensors?",
        "Tell me about digital twins in robotics",
        "What are VLA models?",
        "Explain NVIDIA Isaac platform"
    ]

    print("\n" + "=" * 60)
    print("🔍 Running Test Queries")
    print("=" * 60)

    for query in test_queries:
        print(f"\n📝 Query: \"{query}\"")
        print("-" * 60)

        results = search_module.search(query, limit=3, score_threshold=0.5)

        if results:
            print(f"✅ Found {len(results)} results:\n")
            for result in results:
                print(f"  [{result['rank']}] Score: {result['score']:.2%}")
                print(f"      URL: {result['url']}")
                print(f"      Content: {result['content'][:150]}...")
                print()
        else:
            print("  ❌ No results found\n")

    print("=" * 60)
    print("✅ Testing Complete!")
    print("=" * 60)

if __name__ == "__main__":
    test_search()
