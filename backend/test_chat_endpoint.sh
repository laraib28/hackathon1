#!/bin/bash

echo "======================================"
echo "Testing FastAPI Chat Endpoint"
echo "======================================"
echo ""

# Test health check
echo "1. Testing health endpoint..."
curl -s http://localhost:8000/health | python -m json.tool
echo ""

# Test chat endpoint
echo "2. Testing /api/chat endpoint..."
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ROS2?",
    "target_language": "en"
  }' | python -m json.tool

echo ""
echo "======================================"
echo "If you see a response above, the backend is working correctly!"
echo "======================================"
