# 🚀 Quick Start Guide - Complete System

## ✅ Integration Status: COMPLETE

All frontend and backend components are now fully integrated and working. Follow the steps below to start the entire system.

---

## 📋 Prerequisites

Make sure you have:
- ✅ Node.js installed
- ✅ Python 3.x with uv installed
- ✅ PostgreSQL database (Neon) connection string
- ✅ OpenAI API key (for chatbot)
- ✅ All environment variables configured

---

## 🎯 Start the System (3 Terminals)

### Terminal 1: Auth Backend (Port 3001)

```bash
cd server
npm run dev
```

**Expected Output:**
```
✅ PostgreSQL connected
🚀 Better Auth server running on port 3001
📍 Auth endpoint: http://localhost:3001/api/auth
```

**If you see errors:**
- Check `server/.env` has correct `DATABASE_URL`
- Run `npm install` if packages are missing
- Run `npm run migrate` to create database tables

---

### Terminal 2: FastAPI Backend (Port 8000)

```bash
cd backend
uv run uvicorn main_fastapi:app --reload --port 8000
```

**Expected Output:**
```
🚀 Humanoid Robotics API Starting...
📚 Docs available at: http://localhost:8000/docs
💬 Chat endpoint: /api/chat
🔍 Search endpoint: /api/search
```

**If you see errors:**
- Check `backend/.env` has `OPENAI_API_KEY`
- Check `backend/.env` has `DATABASE_URL`
- Run `uv pip install -r requirements.txt` if packages missing

---

### Terminal 3: Frontend (Port 3000)

```bash
npm start
```

**Expected Output:**
```
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

**If you see errors:**
- Run `npm install` if packages are missing
- Clear cache: `npm run clear`
- Check no other service is using port 3000

---

## 🧪 Test the Integration

### Option 1: Automated Test Script

```bash
./test-integration.sh
```

This will check all endpoints and show you the status.

### Option 2: Manual Testing

#### 1. Test Auth Backend
```bash
curl http://localhost:3001/api/health
```
Should return: `{"status":"ok","message":"Better Auth server is running"}`

#### 2. Test FastAPI Backend
```bash
curl http://localhost:8000/health
```
Should return JSON with status: "healthy"

#### 3. Test Frontend
Open browser: http://localhost:3000

---

## 🎮 Using the Application

### Sign Up / Login

1. Navigate to http://localhost:3000/signup
2. Create an account with:
   - Email
   - Password (min 6 characters)
   - Display name
3. Click "Create Account"
4. You'll be automatically logged in and redirected

**Alternative:** Use http://localhost:3000/login if you already have an account

### Using the Chatbot

#### English Chat:
1. Visit http://localhost:3000 (English site)
2. Click the 💬 button (bottom-right)
3. Type your question: "What is humanoid robotics?"
4. Get response in English

#### Urdu Chat:
1. Switch to Urdu: http://localhost:3000/ur
2. Click the 💬 button
3. Type in Urdu: "یہ کیا ہے؟"
4. Get response in Urdu

#### Text Selection Feature:
1. Select any text on the page
2. Click the "Ask" button that appears
3. Chatbot opens with pre-filled question about selected text

### Language Switching

- **English site**: http://localhost:3000
- **Urdu site**: http://localhost:3000/ur

When you switch languages:
- ✅ UI translates
- ✅ Chatbot welcome message changes
- ✅ Chat history resets (language-specific)
- ✅ Responses come in selected language

---

## 🔍 Key Features Working

### Authentication
- ✅ Signup with email/password
- ✅ Login with email/password
- ✅ Logout functionality
- ✅ Session persistence
- ✅ Protected routes (auto-redirect to login)
- ✅ Google OAuth (if configured)

### Chatbot
- ✅ Connects to FastAPI backend (port 8000)
- ✅ RAG-powered responses (Qdrant + OpenAI)
- ✅ English and Urdu support
- ✅ Site language awareness
- ✅ Chat history with language context
- ✅ Text selection feature
- ✅ Error handling in both languages

### Multi-Language Support
- ✅ URL-based language switching
- ✅ Full UI translation
- ✅ RTL support for Urdu
- ✅ Language-specific chatbot responses
- ✅ Language-aware error messages

---

## 📁 Environment Variables

### Server (.env in `server/`)
```env
DATABASE_URL=postgresql://user:pass@host/neondb
BETTER_AUTH_SECRET=your-secret-key-here
BETTER_AUTH_URL=http://localhost:3001
CLIENT_URL=http://localhost:3000
```

### Backend (.env in `backend/`)
```env
DATABASE_URL=postgresql://user:pass@host/neondb
OPENAI_API_KEY=sk-your-openai-api-key
QDRANT_URL=http://localhost:6333
QDRANT_COLLECTION_NAME=humanoid_robotics
```

### Frontend (optional .env in root)
```env
# Only needed if deploying to production
REACT_APP_API_URL=http://localhost:8000
```

---

## 🐛 Common Issues & Solutions

### Issue: Chatbot says "server not responding"

**Solution:**
1. Check FastAPI is running: `curl http://localhost:8000/health`
2. If not running, start it: `cd backend && uv run uvicorn main_fastapi:app --reload --port 8000`
3. Check `OPENAI_API_KEY` is set in `backend/.env`

### Issue: Login/Signup not working

**Solution:**
1. Check Auth backend is running: `curl http://localhost:3001/api/health`
2. If not running, start it: `cd server && npm run dev`
3. Check `DATABASE_URL` is correct in `server/.env`
4. Run migrations: `cd server && npm run migrate`

### Issue: Language switching not working

**Solution:**
1. Clear browser localStorage: Open DevTools → Application → Local Storage → Clear
2. Clear browser cache
3. Refresh page
4. Verify URL path: `/ur` for Urdu, `/` for English

### Issue: Port already in use

**Solution:**
- Port 3000: `lsof -ti:3000 | xargs kill -9`
- Port 3001: `lsof -ti:3001 | xargs kill -9`
- Port 8000: `lsof -ti:8000 | xargs kill -9`

---

## 📊 System Architecture

```
┌──────────────────────────────────────────────────┐
│  Browser (http://localhost:3000)                │
│  - Docusaurus Frontend                           │
│  - Login/Signup Pages                            │
│  - ChatWidget Component                          │
│  - Language Switcher                             │
└──────────┬─────────────────┬─────────────────────┘
           │                 │
           │                 │
    ┌──────▼──────┐   ┌──────▼──────────────────┐
    │ Auth Server │   │  FastAPI Backend        │
    │ (Port 3001) │   │  (Port 8000)            │
    │             │   │                         │
    │ better-auth │   │  - /api/chat            │
    │ + Express   │   │  - RAG with Qdrant      │
    │             │   │  - OpenAI integration   │
    │ /api/auth/* │   │  - Urdu/English support │
    └──────┬──────┘   └──────┬──────────────────┘
           │                 │
           │                 │
           └────────┬────────┘
                    │
         ┌──────────▼──────────┐
         │  PostgreSQL (Neon)  │
         │  - Users            │
         │  - Sessions         │
         │  - Chat History     │
         └─────────────────────┘
```

---

## ✅ Verification Checklist

Before using the app, verify:

- [ ] Auth backend running on port 3001
- [ ] FastAPI backend running on port 8000
- [ ] Frontend running on port 3000
- [ ] Database migrations completed
- [ ] Environment variables configured
- [ ] Can access http://localhost:3000
- [ ] Can create account at /signup
- [ ] Can login at /login
- [ ] Chatbot responds in English
- [ ] Chatbot responds in Urdu at /ur
- [ ] Language switching works

---

## 🎉 Success Indicators

When everything is working correctly, you should see:

1. **Frontend loads** at http://localhost:3000
2. **Can create account** and login
3. **Chatbot icon** appears bottom-right
4. **Chatbot responds** to English questions
5. **Urdu site** works at /ur
6. **Urdu chatbot** responds in Urdu
7. **Language switch** updates entire interface
8. **Protected routes** redirect to login
9. **Sessions persist** across page refreshes

---

## 📚 Additional Resources

- **API Documentation**: http://localhost:8000/docs
- **Auth Endpoints**: http://localhost:3001/api/auth
- **Integration Guide**: See `INTEGRATION_COMPLETE.md`
- **Test Script**: Run `./test-integration.sh`

---

## 🚀 Ready to Go!

Your complete system is now integrated and ready to use. All components communicate correctly:

- ✅ Frontend → Auth Backend (login/signup)
- ✅ Frontend → FastAPI Backend (chatbot)
- ✅ Both Backends → PostgreSQL Database
- ✅ FastAPI → OpenAI (chat responses)
- ✅ FastAPI → Qdrant (RAG context)

**The system is production-ready for your hackathon demo!**

Start all three terminals as shown above and enjoy your fully functional application! 🎊
