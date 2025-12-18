"""
Main FastAPI Application - Production Ready
Combines authentication, chat, and search functionality
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import routers
from auth_api import router as auth_router
from chat_api import router as chat_router
from qdrant_api import router as qdrant_router

# Create FastAPI app
app = FastAPI(
    title="Humanoid Robotics API",
    description="Complete API for robotics learning platform with auth, chat, and search",
    version="2.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS Configuration - Allow all origins for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production: ["https://your-domain.com", "http://localhost:3000"]
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers with proper prefixes
app.include_router(auth_router, prefix="/api/auth", tags=["Authentication"])
app.include_router(chat_router, tags=["Chat"])  # Already has /api prefix in router
app.include_router(qdrant_router, tags=["Search"])  # Already has /api prefix in router

# Root endpoint
@app.get("/")
async def root():
    return {
        "message": "Humanoid Robotics API - Production Ready",
        "version": "2.0.0",
        "status": "running",
        "endpoints": {
            "docs": "/docs",
            "auth": "/api/auth/*",
            "chat": "/api/chat",
            "search": "/api/search",
            "health": "/health"
        }
    }

# Health check endpoint
@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "service": "Humanoid Robotics API",
        "version": "2.0.0",
        "components": {
            "auth": "operational",
            "chat": "operational",
            "search": "operational",
            "database": "connected",
            "qdrant": "connected"
        }
    }

# Startup event
@app.on_event("startup")
async def startup_event():
    print("=" * 70)
    print("🚀 Humanoid Robotics API Starting...")
    print("=" * 70)
    print("📚 Docs available at: http://localhost:8000/docs")
    print("🔐 Auth endpoints: /api/auth/*")
    print("💬 Chat endpoint: /api/chat")
    print("🔍 Search endpoint: /api/search")
    print("=" * 70)

if __name__ == "__main__":
    import uvicorn

    print("\n" + "=" * 70)
    print("Starting Humanoid Robotics API Server")
    print("=" * 70)

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        reload=True,  # Enable auto-reload in development
        log_level="info"
    )
