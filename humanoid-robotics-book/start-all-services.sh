#!/bin/bash

# Complete Development Startup Script
# Starts ALL THREE services: Frontend, Auth Server, and Chat Backend

echo "🚀 Starting ALL Services for Humanoid Robotics Book"
echo ""
echo "This will start:"
echo "  1. Frontend (Docusaurus) - Port 3000"
echo "  2. Auth Server (Better Auth) - Port 3001"
echo "  3. Chat Backend (FastAPI) - Port 8000"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if server/.env exists
if [ ! -f "server/.env" ]; then
  echo -e "${YELLOW}⚠️  Warning: server/.env not found${NC}"
  echo "   Creating from server/.env.example..."
  cp server/.env.example server/.env
  echo -e "   ${RED}⚠️  IMPORTANT: Edit server/.env with your database credentials!${NC}"
  echo ""
fi

# Check if .env.local exists
if [ ! -f ".env.local" ]; then
  echo -e "${YELLOW}⚠️  Warning: .env.local not found${NC}"
  echo "   Creating from .env.example..."
  cp .env.example .env.local
  echo ""
fi

# Check if backend exists
if [ ! -d "../backend" ]; then
  echo -e "${RED}❌ Error: ../backend directory not found!${NC}"
  echo "   The FastAPI chat backend should be in ../backend"
  echo "   Current directory: $(pwd)"
  exit 1
fi

# Check if backend has required files
if [ ! -f "../backend/main_fastapi.py" ]; then
  echo -e "${RED}❌ Error: ../backend/main_fastapi.py not found!${NC}"
  echo "   Cannot start chat backend"
  exit 1
fi

# Check for Python
if ! command -v python3 &> /dev/null; then
  echo -e "${RED}❌ Error: python3 not found!${NC}"
  echo "   Please install Python 3.8+ to run the chat backend"
  exit 1
fi

echo "📦 Installing dependencies (if needed)..."
echo ""

# Install root dependencies
if [ ! -d "node_modules" ]; then
  echo "Installing frontend dependencies..."
  npm install
fi

# Install server dependencies
if [ ! -d "server/node_modules" ]; then
  echo "Installing server dependencies..."
  cd server && npm install && cd ..
fi

# Check backend Python dependencies
echo "Checking backend dependencies..."
cd ../backend
if [ ! -f "requirements.txt" ]; then
  echo -e "${YELLOW}⚠️  Warning: requirements.txt not found in backend${NC}"
else
  # Try to check if dependencies are installed
  python3 -c "import fastapi, openai, qdrant_client" 2>/dev/null
  if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠️  Backend dependencies may be missing${NC}"
    echo "   To install: cd ../backend && pip3 install -r requirements.txt"
    echo ""
  fi
fi
cd ../humanoid-robotics-book

echo ""
echo -e "${GREEN}✅ Dependencies ready${NC}"
echo ""
echo "🔑 Starting Better Auth Server (port 3001)..."
echo "💬 Starting FastAPI Chat Backend (port 8000)..."
echo "📚 Starting Docusaurus Frontend (port 3000)..."
echo ""
echo "───────────────────────────────────────────────"
echo "  Auth Server:    http://localhost:3001"
echo "  Health Check:   http://localhost:3001/api/health"
echo "  Chat Backend:   http://localhost:8000"
echo "  Backend Docs:   http://localhost:8000/docs"
echo "  Frontend:       http://localhost:3000"
echo "───────────────────────────────────────────────"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all servers${NC}"
echo ""

# Cleanup function to kill all background jobs
cleanup() {
  echo ""
  echo "🛑 Stopping all services..."
  kill $(jobs -p) 2>/dev/null
  exit 0
}

# Set trap to cleanup on exit
trap cleanup EXIT INT TERM

# Start all three servers in parallel

# 1. Start Auth Server (port 3001)
echo -e "${GREEN}[AUTH]${NC} Starting..."
(cd server && npm run dev) > /tmp/auth-server.log 2>&1 &
AUTH_PID=$!

# 2. Start Chat Backend (port 8000)
echo -e "${GREEN}[CHAT]${NC} Starting..."
(cd ../backend && python3 main_fastapi.py) > /tmp/chat-backend.log 2>&1 &
CHAT_PID=$!

# Wait a moment for backends to start
sleep 3

# 3. Start Frontend (port 3000)
echo -e "${GREEN}[FRONTEND]${NC} Starting..."
npm run start:frontend &
FRONTEND_PID=$!

echo ""
echo -e "${GREEN}✅ All services started!${NC}"
echo ""
echo "📋 To view logs:"
echo "   Auth Server: tail -f /tmp/auth-server.log"
echo "   Chat Backend: tail -f /tmp/chat-backend.log"
echo ""
echo "🔍 Quick health checks:"
echo "   curl http://localhost:3001/api/health  # Auth"
echo "   curl http://localhost:8000/health      # Chat Backend"
echo ""
echo "💡 Open http://localhost:3000 in your browser"
echo ""

# Monitor processes and restart if they die
while true; do
  # Check if auth server is still running
  if ! kill -0 $AUTH_PID 2>/dev/null; then
    echo -e "${RED}❌ Auth server died! Check /tmp/auth-server.log${NC}"
    cleanup
  fi

  # Check if chat backend is still running
  if ! kill -0 $CHAT_PID 2>/dev/null; then
    echo -e "${RED}❌ Chat backend died! Check /tmp/chat-backend.log${NC}"
    cleanup
  fi

  # Check if frontend is still running
  if ! kill -0 $FRONTEND_PID 2>/dev/null; then
    echo -e "${RED}❌ Frontend died!${NC}"
    cleanup
  fi

  sleep 5
done
