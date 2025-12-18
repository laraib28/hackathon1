# 🚀 COMPLETE STARTUP GUIDE - Production Ready

## ✅ What's Been Fixed

| Component | Status | File |
|-----------|--------|------|
| Main FastAPI App | ✅ Fixed | `main_fastapi.py` |
| Auth Router | ✅ Fixed | `auth_api.py` |
| Chat Router | ✅ Fixed | `chat_api.py` |
| Qdrant Search | ✅ Working | `qdrant_api.py` |
| Database | ✅ Working | `database.py` |
| Embeddings | ✅ Working | `embedding_service.py` |
| Search | ✅ Working | `search.py` |
| ChatWidget | ✅ Fixed | `ChatWidget.tsx` |
| Dependencies | ✅ Complete | `requirements.txt` |
| Environment | ✅ Documented | `.env.example` |

## 📋 Prerequisites

- Python 3.12+
- Node.js 18+
- PostgreSQL (Neon account)
- Qdrant Cloud account
- OpenAI API key

## 🔧 Backend Setup

### Step 1: Navigate to Backend Directory

```bash
cd backend
```

### Step 2: Create Virtual Environment

```bash
python -m venv .venv
```

### Step 3: Activate Virtual Environment

**On Linux/Mac:**
```bash
source .venv/bin/activate
```

**On Windows:**
```bash
.venv\Scripts\activate
```

### Step 4: Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

### Step 5: Create .env File

```bash
cp .env.example .env
```

Then edit `.env` with your actual credentials:

```env
OPENAI_API_KEY=sk-your-actual-key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-actual-key
DATABASE_URL=postgresql://user:pass@host/db
```

### Step 6: Initialize Database

```bash
python database.py
```

Expected output:
```
======================================================================
🗄️  Initializing Neon Postgres Database
======================================================================
✅ Database tables created successfully
✅ Database setup complete!
```

### Step 7: Populate Embeddings (if needed)

```bash
python check_embeddings.py
```

If it shows 0 vectors:
```bash
python populate_embeddings.py
```

Expected output:
```
✅ Prepared 1012 document chunks
✅ Batch 1 inserted (10 points)
...
✅ COMPLETE! Embeddings successfully populated in Qdrant
📊 Total vectors: 1012
```

### Step 8: Start Backend Server

```bash
python main_fastapi.py
```

Expected output:
```
======================================================================
🚀 Humanoid Robotics API Starting...
======================================================================
📚 Docs available at: http://localhost:8000/docs
🔐 Auth endpoints: /api/auth/*
💬 Chat endpoint: /api/chat
🔍 Search endpoint: /api/search
======================================================================
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Step 9: Verify Backend is Running

Open a new terminal and test:

```bash
curl http://localhost:8000/health
```

Expected response:
```json
{
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
```

### Step 10: Test Chat Endpoint

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS2?", "target_language": "en"}'
```

Expected response:
```json
{
  "response": "ROS2 is...",
  "sources": [...],
  "language": "en"
}
```

## 🎨 Frontend Setup

### Step 1: Navigate to Frontend Directory

```bash
cd ../humanoid-robotics-book
```

### Step 2: Install Dependencies

```bash
npm install
```

### Step 3: Verify .env.local Exists

Check if `.env.local` exists:
```bash
cat .env.local
```

Should contain:
```env
REACT_APP_API_URL=http://localhost:8000
```

If not, create it:
```bash
echo "REACT_APP_API_URL=http://localhost:8000" > .env.local
```

### Step 4: Start Frontend Development Server

```bash
npm start
```

Expected output:
```
Compiled successfully!

You can now view humanoid-robotics-book in the browser.

  Local:            http://localhost:3000
  On Your Network:  http://192.168.x.x:3000
```

### Step 5: Test Frontend

1. Open browser: http://localhost:3000
2. Click chat widget button (💬)
3. Send message: "What is ROS2?"
4. Check browser console (F12) for:

```
🔄 Sending POST request to: http://localhost:8000/api/chat
📦 Request body: {...}
📡 Response status: 200
✅ Response received: {...}
```

## 🧪 Complete System Test

### Terminal 1 - Backend
```bash
cd backend
source .venv/bin/activate
python main_fastapi.py
```

### Terminal 2 - Frontend
```bash
cd humanoid-robotics-book
npm start
```

### Terminal 3 - Testing
```bash
# Test backend health
curl http://localhost:8000/health

# Test chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "test", "target_language": "en"}'

# Test auth signup
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "test123",
    "name": "Test User",
    "software_experience": "beginner",
    "hardware_experience": "none",
    "programming_level": "beginner",
    "programming_languages": ["Python"],
    "learning_goals": "Learn robotics"
  }'
```

## 🐛 Troubleshooting

### Backend Won't Start

**Error:** `ModuleNotFoundError: No module named 'fastapi'`

**Solution:**
```bash
# Ensure virtual environment is activated
source .venv/bin/activate
# Reinstall dependencies
pip install -r requirements.txt
```

**Error:** `Cannot import router from auth_api`

**Solution:** Already fixed! auth_api.py now properly exports `router = APIRouter()`

**Error:** `email-validator missing`

**Solution:**
```bash
pip install email-validator
```

### Chat Returns 404

**Problem:** POST /chat returns 404

**Solution:** Already fixed! Chat endpoint is at `/api/chat` (line 48 in chat_api.py)

**Verify:**
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "test", "target_language": "en"}'
```

### CORS Errors

**Problem:** Frontend can't reach backend

**Solution:** CORS is already configured in `main_fastapi.py:29-35` to allow all origins

**Verify:** Check backend logs for CORS errors

### Database Connection Failed

**Problem:** Cannot connect to Neon Postgres

**Solution:**
1. Check DATABASE_URL in `.env`
2. Verify Neon project is active
3. Test connection:
```bash
python -c "from database import engine; print(engine.url)"
```

### Qdrant Search Returns Empty

**Problem:** No search results

**Solution:**
1. Check embeddings exist:
```bash
python check_embeddings.py
```

2. If 0 vectors, populate:
```bash
python populate_embeddings.py
```

3. Lower score threshold in `chat_api.py:79` from 0.5 to 0.3

## 📚 API Documentation

Once backend is running, visit:

**Swagger UI:** http://localhost:8000/docs

**ReDoc:** http://localhost:8000/redoc

## 🔐 API Endpoints

### Authentication
- `POST /api/auth/signup` - Register new user
- `POST /api/auth/login` - Login user
- `GET /api/auth/me` - Get current user
- `POST /api/auth/logout` - Logout user
- `PUT /api/auth/profile` - Update profile

### Chat
- `POST /api/chat` - Send chat message (RAG-powered)

### Search
- `POST /api/search` - Semantic search
- `GET /api/qdrant/stats` - Collection statistics

### System
- `GET /` - API information
- `GET /health` - Health check

## 🚀 Production Deployment

### Backend (Railway/Render/Fly.io)

1. Push code to GitHub
2. Connect repository to hosting platform
3. Set environment variables:
   - `OPENAI_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `DATABASE_URL`
4. Deploy
5. Note production URL (e.g., `https://my-api.railway.app`)

### Frontend (Vercel)

1. Push code to GitHub
2. Connect repository to Vercel
3. Set environment variable:
   - `REACT_APP_API_URL=https://my-api.railway.app`
4. Deploy
5. Test chat on production site

## ✅ Success Checklist

- [ ] Backend runs on http://localhost:8000
- [ ] `/health` returns healthy status
- [ ] `/api/chat` accepts POST and returns response
- [ ] Swagger docs load at `/docs`
- [ ] Frontend runs on http://localhost:3000
- [ ] Chat widget opens
- [ ] Sending message gets response (not error)
- [ ] Browser console shows 200 status
- [ ] No CORS errors in console

## 🎉 You're Done!

Your full-stack robotics learning platform is now running!

**Backend:** http://localhost:8000
**Frontend:** http://localhost:3000
**API Docs:** http://localhost:8000/docs

Need help? Check the API docs or inspect browser/server logs.
