"""
Simple API for Semantic Search
Can be run with Flask or FastAPI
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import search as search_module

app = Flask(__name__)
CORS(app)  # Enable CORS for frontend access

@app.route('/api/search', methods=['POST'])
def api_search():
    """
    Search endpoint

    POST /api/search
    Body: {
        "query": "your search query",
        "limit": 5,
        "score_threshold": 0.7
    }
    """
    try:
        data = request.get_json()

        if not data or 'query' not in data:
            return jsonify({
                "success": False,
                "error": "Query parameter is required"
            }), 400

        query = data['query']
        limit = data.get('limit', 5)
        score_threshold = data.get('score_threshold', 0.7)

        # Perform search
        results = search_module.search(query, limit, score_threshold)

        return jsonify({
            "success": True,
            "query": query,
            "results": results,
            "count": len(results)
        })

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/stats', methods=['GET'])
def api_stats():
    """
    Get collection statistics

    GET /api/stats
    """
    try:
        stats = search_module.get_collection_stats()
        return jsonify({
            "success": True,
            "stats": stats
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/health', methods=['GET'])
def health():
    """
    Health check endpoint
    """
    return jsonify({"status": "healthy"})

if __name__ == '__main__':
    print("=" * 60)
    print("🚀 Starting Semantic Search API")
    print("=" * 60)
    print("\nEndpoints:")
    print("  POST /api/search - Perform semantic search")
    print("  GET  /api/stats  - Get collection statistics")
    print("  GET  /health     - Health check")
    print("\nRunning on: http://localhost:5000")
    print("=" * 60)

    app.run(host='0.0.0.0', port=5000, debug=True)
