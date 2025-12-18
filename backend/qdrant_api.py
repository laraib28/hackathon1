"""
Qdrant Search Integration with User System
FastAPI endpoints for semantic search with user tracking
"""

from fastapi import APIRouter, Depends, HTTPException, BackgroundTasks
from pydantic import BaseModel
from typing import List, Optional, Dict
import time
from datetime import datetime

from database import get_db, get_user_by_id, ChatHistory
from embedding_service import get_embedding_service
from search import search as qdrant_search
from auth_api import require_auth
import os
from qdrant_client import QdrantClient

router = APIRouter(prefix="/api")

# Initialize Qdrant client
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Pydantic models
class SearchRequest(BaseModel):
    query: str
    user_id: Optional[str] = None
    limit: int = 5
    score_threshold: float = 0.3

class SearchResponse(BaseModel):
    query: str
    results: List[Dict]
    search_id: str
    timestamp: datetime

class QdrantStatsResponse(BaseModel):
    collection_name: str
    vectors_count: int
    vector_size: int
    user_id: Optional[str] = None

@router.post("/search", response_model=SearchResponse)
async def search_endpoint(
    request: SearchRequest,
    db=Depends(get_db),
    current_user: Dict = Depends(require_auth)
):
    """
    Perform semantic search using Qdrant with optional user tracking
    """
    start_time = time.time()
    
    # Validate user if provided
    user = None
    if request.user_id:
        user = get_user_by_id(db, request.user_id)
        if not user:
            raise HTTPException(status_code=404, detail="User not found")
    
    # Perform search using Qdrant
    try:
        results = qdrant_search(
            query=request.query,
            limit=request.limit,
            score_threshold=request.score_threshold
        )
        
        search_duration = time.time() - start_time
        
        # Log search to database if user is authenticated
        if user:
            # Save to chat history for analytics
            chat_history = ChatHistory(
                user_id=request.user_id,
                message=request.query,
                response=f"Qdrant search returned {len(results)} results",
                language="en",
                sources=[{"url": r["url"], "score": r["score"]} for r in results]
            )
            db.add(chat_history)
            db.commit()
        
        # Create search response
        response = SearchResponse(
            query=request.query,
            results=results,
            search_id=f"search_{int(time.time())}",
            timestamp=datetime.utcnow()
        )
        
        return response
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Search error: {str(e)}")


@router.get("/qdrant/stats", response_model=QdrantStatsResponse)
async def get_qdrant_stats(db=Depends(get_db)):
    """
    Get Qdrant collection statistics
    """
    try:
        from search import COLLECTION_NAME
        
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        
        stats = QdrantStatsResponse(
            collection_name=COLLECTION_NAME,
            vectors_count=collection_info.points_count,
            vector_size=collection_info.config.params.vectors.size,
            user_id=None  # This is system-wide info, not user-specific
        )
        
        return stats
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Qdrant stats error: {str(e)}")


@router.get("/user/search-history/{user_id}")
async def get_user_search_history(
    user_id: str,
    limit: int = 10,
    db=Depends(get_db),
    current_user: Dict = Depends(require_auth)
):
    """
    Get search history for a specific user
    """
    user = get_user_by_id(db, user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    
    # Get chat history which includes search queries
    search_history = db.query(ChatHistory).filter(
        ChatHistory.user_id == user_id
    ).order_by(ChatHistory.created_at.desc()).limit(limit).all()
    
    return {
        "user_id": user_id,
        "search_history": [
            {
                "id": item.id,
                "query": item.message,
                "response_preview": item.response[:100] + "..." if len(item.response) > 100 else item.response,
                "sources": item.sources,
                "timestamp": item.created_at
            }
            for item in search_history
        ]
    }


# Background task to sync embeddings periodically
def sync_qdrant_with_user_content(user_id: str, content: str):
    """
    Background task to add user-specific content to Qdrant if needed
    """
    # This is a placeholder for future functionality to index user-specific content
    pass


if __name__ == "__main__":
    # Test the search functionality
    import asyncio
    
    async def test_search():
        request = SearchRequest(query="humanoid robotics", limit=3)
        result = await search_endpoint(request)
        print(f"Test search result: {len(result.results)} results")
    
    # asyncio.run(test_search())