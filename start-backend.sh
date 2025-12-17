#!/bin/bash

# Humanoid Robotics Book - Backend Startup Script
# This script starts all backend services

echo "=========================================="
echo "🚀 Starting Backend Services"
echo "=========================================="
echo ""

# Check if we're in the correct directory
if [ ! -d "backend" ]; then
    echo "❌ Error: backend directory not found!"
    echo "Please run this script from the hackathon1 root directory"
    exit 1
fi

cd backend

# Check if .env file exists
if [ ! -f ".env" ]; then
    echo "⚠️  Warning: .env file not found!"
    echo "Please create backend/.env with required credentials:"
    echo "  - OPENAI_API_KEY"
    echo "  - QDRANT_URL"
    echo "  - QDRANT_API_KEY"
    echo "  - DATABASE_URL"
    exit 1
fi

echo "✅ Environment file found"
echo ""

# Check if virtual environment exists
if [ ! -d ".venv" ]; then
    echo "📦 Creating virtual environment..."
    python3 -m venv .venv
fi

echo "🔄 Activating virtual environment..."
source .venv/bin/activate

echo "📦 Installing dependencies..."
pip install -q -e .

echo ""
echo "=========================================="
echo "Starting Services:"
echo "  - Chat API (FastAPI): http://localhost:8000"
echo "  - Auth API (FastAPI): http://localhost:8001"
echo "=========================================="
echo ""

# Start Chat API in background
echo "🚀 Starting Chat API on port 8000..."
python chat_api.py &
CHAT_PID=$!

# Wait a bit for the first service to start
sleep 2

# Start Auth API in background
echo "🚀 Starting Auth API on port 8001..."
python auth_api.py &
AUTH_PID=$!

echo ""
echo "=========================================="
echo "✅ Backend services started!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  Chat API: $CHAT_PID"
echo "  Auth API: $AUTH_PID"
echo ""
echo "To stop services:"
echo "  kill $CHAT_PID $AUTH_PID"
echo ""
echo "Or press Ctrl+C to stop all services"
echo "=========================================="

# Wait for all background processes
wait
