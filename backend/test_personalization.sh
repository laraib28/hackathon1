#!/bin/bash

echo "======================================"
echo "Testing Personalization Endpoint"
echo "======================================"
echo ""

# Test health check
echo "1. Testing health endpoint..."
curl -s http://localhost:8000/health | python -m json.tool
echo ""

# Test personalization endpoint
echo "2. Testing /api/chat/personalize endpoint..."
curl -X POST http://localhost:8000/api/chat/personalize \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Physical AI is an interdisciplinary field that combines robotics, machine learning, and control theory to create intelligent physical systems that can interact with the real world.",
    "user_background": {
      "software_experience": "beginner",
      "hardware_experience": "none",
      "programming_level": "beginner",
      "learning_goals": "Learn robotics basics"
    },
    "chapter_id": "ch1_introduction",
    "user_id": "test_user_123"
  }' | python -m json.tool

echo ""
echo "======================================"
echo "3. Testing /api/chat/translate endpoint..."
curl -X POST http://localhost:8000/api/chat/translate \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Physical AI is an interdisciplinary field that combines robotics, machine learning, and control theory.",
    "target_language": "ur"
  }' | python -m json.tool

echo ""
echo "======================================"
echo "If you see successful responses above, the personalization endpoints are working correctly!"
echo "======================================"