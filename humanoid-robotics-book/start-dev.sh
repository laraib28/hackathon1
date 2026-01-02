#!/bin/bash

# Development startup script for Humanoid Robotics Book
# Starts both the auth server and Docusaurus frontend

echo "🚀 Starting Humanoid Robotics Book Development Environment"
echo ""

# Check if server/.env exists
if [ ! -f "server/.env" ]; then
  echo "⚠️  Warning: server/.env not found"
  echo "   Creating from server/.env.example..."
  cp server/.env.example server/.env
  echo "   ⚠️  IMPORTANT: Edit server/.env with your database credentials and secrets!"
  echo ""
fi

# Check if .env.local exists
if [ ! -f ".env.local" ]; then
  echo "⚠️  Warning: .env.local not found"
  echo "   Creating from .env.example..."
  cp .env.example .env.local
  echo ""
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

echo ""
echo "✅ Dependencies ready"
echo ""
echo "🔑 Starting Better Auth Server (port 3001)..."
echo "📚 Starting Docusaurus Frontend (port 3000)..."
echo ""
echo "───────────────────────────────────────────────"
echo "  Auth Server:    http://localhost:3001"
echo "  Health Check:   http://localhost:3001/api/health"
echo "  Frontend:       http://localhost:3000"
echo "───────────────────────────────────────────────"
echo ""
echo "Press Ctrl+C to stop both servers"
echo ""

# Start both servers in parallel
# The trap ensures both are killed when this script exits
trap 'kill $(jobs -p) 2>/dev/null' EXIT

# Start backend
(cd server && npm run dev) &

# Wait a moment for backend to start
sleep 2

# Start frontend
npm run start:frontend &

# Wait for both background jobs
wait
