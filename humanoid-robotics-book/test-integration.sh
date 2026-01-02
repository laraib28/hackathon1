#!/bin/bash

echo "=================================="
echo "🧪 Testing Full-Stack Integration"
echo "=================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test Auth Backend
echo "1️⃣  Testing Auth Backend (Port 3001)..."
AUTH_RESPONSE=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:3001/api/health)
if [ "$AUTH_RESPONSE" = "200" ]; then
    echo -e "${GREEN}✅ Auth Backend: HEALTHY${NC}"
    curl -s http://localhost:3001/api/health | jq '.'
else
    echo -e "${RED}❌ Auth Backend: NOT RESPONDING (HTTP $AUTH_RESPONSE)${NC}"
    echo "   Make sure to run: cd server && npm run dev"
fi
echo ""

# Test FastAPI Backend
echo "2️⃣  Testing FastAPI Backend (Port 8000)..."
API_RESPONSE=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8000/health)
if [ "$API_RESPONSE" = "200" ]; then
    echo -e "${GREEN}✅ FastAPI Backend: HEALTHY${NC}"
    curl -s http://localhost:8000/health | jq '.'
else
    echo -e "${RED}❌ FastAPI Backend: NOT RESPONDING (HTTP $API_RESPONSE)${NC}"
    echo "   Make sure to run: cd backend && uv run uvicorn main_fastapi:app --reload --port 8000"
fi
echo ""

# Test Frontend
echo "3️⃣  Testing Frontend (Port 3000)..."
FRONTEND_RESPONSE=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:3000)
if [ "$FRONTEND_RESPONSE" = "200" ]; then
    echo -e "${GREEN}✅ Frontend: RUNNING${NC}"
else
    echo -e "${RED}❌ Frontend: NOT RESPONDING (HTTP $FRONTEND_RESPONSE)${NC}"
    echo "   Make sure to run: npm start"
fi
echo ""

# Test Chat API
echo "4️⃣  Testing Chat API Endpoint..."
if [ "$API_RESPONSE" = "200" ]; then
    CHAT_TEST=$(curl -s -X POST http://localhost:8000/api/chat \
        -H "Content-Type: application/json" \
        -d '{
            "message": "Hello",
            "target_language": "en"
        }')

    if echo "$CHAT_TEST" | jq -e '.response' > /dev/null 2>&1; then
        echo -e "${GREEN}✅ Chat API: WORKING${NC}"
        echo "   Response preview:"
        echo "$CHAT_TEST" | jq -r '.response' | head -c 100
        echo "..."
    else
        echo -e "${RED}❌ Chat API: ERROR${NC}"
        echo "$CHAT_TEST" | jq '.'
    fi
else
    echo -e "${YELLOW}⚠️  Skipping (FastAPI not running)${NC}"
fi
echo ""

# Summary
echo "=================================="
echo "📊 Integration Status Summary"
echo "=================================="
echo ""

TOTAL_TESTS=3
PASSED_TESTS=0

if [ "$AUTH_RESPONSE" = "200" ]; then
    ((PASSED_TESTS++))
fi

if [ "$API_RESPONSE" = "200" ]; then
    ((PASSED_TESTS++))
fi

if [ "$FRONTEND_RESPONSE" = "200" ]; then
    ((PASSED_TESTS++))
fi

echo "Tests Passed: $PASSED_TESTS/$TOTAL_TESTS"

if [ $PASSED_TESTS -eq $TOTAL_TESTS ]; then
    echo -e "${GREEN}✅ ALL SYSTEMS OPERATIONAL${NC}"
    echo ""
    echo "🎉 Your application is ready!"
    echo ""
    echo "Access Points:"
    echo "  • Frontend:  http://localhost:3000"
    echo "  • Frontend (Urdu): http://localhost:3000/ur"
    echo "  • Auth API:  http://localhost:3001/api/auth"
    echo "  • Chat API:  http://localhost:8000/api/chat"
    echo "  • API Docs:  http://localhost:8000/docs"
else
    echo -e "${RED}⚠️  SOME SYSTEMS NOT RUNNING${NC}"
    echo ""
    echo "To start all services:"
    echo ""
    echo "Terminal 1 - Auth Backend:"
    echo "  cd server && npm run dev"
    echo ""
    echo "Terminal 2 - FastAPI Backend:"
    echo "  cd backend && uv run uvicorn main_fastapi:app --reload --port 8000"
    echo ""
    echo "Terminal 3 - Frontend:"
    echo "  npm start"
fi

echo ""
echo "=================================="
