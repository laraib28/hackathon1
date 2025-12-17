#!/bin/bash

# Backend Test Script - Production Ready
# Tests all endpoints to verify backend is working

echo "======================================"
echo "🧪 Testing Backend API"
echo "======================================"
echo ""

BASE_URL="http://localhost:8000"

# Test 1: Root endpoint
echo "1️⃣  Testing root endpoint..."
curl -s $BASE_URL | python -m json.tool
echo ""

# Test 2: Health check
echo "2️⃣  Testing health endpoint..."
curl -s $BASE_URL/health | python -m json.tool
echo ""

# Test 3: Chat endpoint
echo "3️⃣  Testing chat endpoint (POST /api/chat)..."
curl -X POST $BASE_URL/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is ROS2?",
    "target_language": "en"
  }' | python -m json.tool
echo ""

# Test 4: Auth signup
echo "4️⃣  Testing auth signup..."
curl -X POST $BASE_URL/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test-'$(date +%s)'@example.com",
    "password": "test123",
    "name": "Test User",
    "software_experience": "beginner",
    "hardware_experience": "none",
    "programming_level": "beginner",
    "programming_languages": ["Python"],
    "learning_goals": "Learn robotics"
  }' | python -m json.tool
echo ""

# Test 5: Qdrant stats
echo "5️⃣  Testing Qdrant stats..."
curl -s $BASE_URL/api/qdrant/stats | python -m json.tool
echo ""

echo "======================================"
echo "✅ All tests complete!"
echo "======================================"
echo ""
echo "If all tests passed, backend is working correctly!"
echo "Next: Start frontend with 'npm start'"
