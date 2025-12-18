# Quickstart Guide: Book UI Enhancement, Better Auth Integration, and Urdu Translation Fix

**Feature**: 003-book-ui-auth-urdu
**Date**: 2025-12-18
**Audience**: Developers setting up local environment or deploying to production

## Prerequisites

Before starting, ensure you have:

- **Node.js** 20.x or higher ([Download](https://nodejs.org/))
- **Python** 3.11 or higher ([Download](https://www.python.org/downloads/))
- **Git** ([Download](https://git-scm.com/downloads))
- **Code editor** (VS Code recommended)
- **Accounts**:
  - Qdrant Cloud account (for vector database)
  - OpenAI API key (for chatbot translation)
  - Vercel account (for frontend deployment)
  - Railway account (for backend deployment)

---

## Quick Start (5 Minutes)

### 1. Clone Repository

```bash
git clone <repository-url>
cd humanoid-robotics-book
git checkout 003-book-ui-auth-urdu
```

### 2. Frontend Setup

```bash
# Install dependencies
npm install

# Create environment file
cat > .env.local << 'EOF'
REACT_APP_API_URL=http://localhost:8000
EOF

# Start development server
npm start
```

Frontend will run at http://localhost:3000

### 3. Backend Setup

```bash
cd ../backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create environment file
cat > .env << 'EOF'
# Database
DATABASE_URL=sqlite:///./app.db

# Authentication
BETTER_AUTH_SECRET=your-secret-key-min-32-chars-long-change-this
BETTER_AUTH_URL=http://localhost:8000

# OpenAI
OPENAI_API_KEY=sk-your-openai-api-key-here

# Qdrant Vector Database
QDRANT_URL=https://your-cluster.qdrant.tech
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=humanoid_robotics_docs
EOF

# Initialize database
python -c "from database import init_db; init_db()"

# Start backend server
uvicorn main_fastapi:app --reload
```

Backend will run at http://localhost:8000

### 4. Test the Setup

1. Open http://localhost:3000
2. Click "Sign Up" and create an account
3. Open chatbot and ask: "What is a humanoid robot?"
4. Try Urdu query: "Robotics kya hai?"

---

## Detailed Setup

### Frontend (Docusaurus)

#### Install Dependencies

```bash
cd humanoid-robotics-book
npm install
```

**Key dependencies**:
- `@docusaurus/core@3.9.2` - Static site generator
- `better-auth@1.4.7` - Authentication library
- `axios@1.7.0` - HTTP client
- `react@19.0.0` - UI framework

#### Environment Variables

Create `.env.local` in `humanoid-robotics-book/` directory:

```env
# Backend API URL
REACT_APP_API_URL=http://localhost:8000
```

**Production** (Vercel):
```env
REACT_APP_API_URL=https://hackathon1-production-aaf0.up.railway.app
```

#### Development Commands

```bash
# Start dev server (hot reload)
npm start

# Build for production
npm run build

# Serve production build locally
npm run serve

# Type check
npm run typecheck

# Clear cache
npm run clear
```

#### Project Structure

```
humanoid-robotics-book/
├── docs/               # Book content (Markdown files)
├── src/
│   ├── components/     # React components
│   │   ├── Auth/       # Login, Signup, ProtectedRoute
│   │   ├── ChatWidget/ # Chatbot UI
│   │   └── ...
│   ├── pages/          # Custom pages (index, login, signup, profile)
│   ├── context/        # Auth context provider
│   ├── services/       # API service (authService.ts)
│   ├── utils/          # Utilities (languageDetector.ts)
│   ├── css/            # Global styles (custom.css, rtl.css)
│   └── theme/          # Theme customization
├── static/             # Static assets
├── docusaurus.config.ts # Docusaurus configuration
├── sidebars.ts         # Sidebar navigation
└── package.json        # Dependencies
```

---

### Backend (FastAPI)

#### Install Dependencies

```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

**Key dependencies**:
- `fastapi` - Web framework
- `uvicorn` - ASGI server
- `sqlalchemy` - ORM for database
- `openai` - OpenAI API client
- `qdrant-client` - Vector database client
- `sentence-transformers` - Embedding model

#### Environment Variables

Create `.env` in `backend/` directory:

```env
# Database
DATABASE_URL=sqlite:///./app.db

# Authentication
BETTER_AUTH_SECRET=generate-a-32-char-secret-key-using-openssl-rand-hex-32
BETTER_AUTH_URL=http://localhost:8000

# OpenAI API
OPENAI_API_KEY=sk-proj-your-actual-api-key-from-platform-openai-com

# Qdrant Vector Database
QDRANT_URL=https://your-qdrant-cluster.qdrant.tech:6333
QDRANT_API_KEY=your-qdrant-api-key-from-dashboard
QDRANT_COLLECTION_NAME=humanoid_robotics_docs

# CORS (Production)
ALLOWED_ORIGINS=https://hackathon1-9y2e.vercel.app,http://localhost:3000
```

**Generate BETTER_AUTH_SECRET**:
```bash
openssl rand -hex 32
```

#### Initialize Database

```bash
python -c "from database import init_db; init_db()"
```

This creates `app.db` with `users` and `chat_history` tables.

#### Populate Qdrant Embeddings

**Note**: This step is only needed if vector database is empty.

```bash
# Edit main.py to set SITEMAP_URL to your Docusaurus deployment
python main.py
```

This scrapes the book sitemap and uploads embeddings to Qdrant.

#### Development Commands

```bash
# Start dev server (hot reload)
uvicorn main_fastapi:app --reload

# Start production server
uvicorn main_fastapi:app --host 0.0.0.0 --port 8000

# Run with specific workers (production)
gunicorn main_fastapi:app --workers 4 --worker-class uvicorn.workers.UvicornWorker
```

#### API Endpoints

Once running, visit:
- **API Docs**: http://localhost:8000/docs (Swagger UI)
- **Health Check**: http://localhost:8000/
- **Auth Endpoints**: http://localhost:8000/api/auth/*
- **Chat Endpoint**: http://localhost:8000/api/chat

---

## Testing

### Manual Testing Checklist

#### Authentication Flow

1. **Signup**:
   - Navigate to http://localhost:3000/signup
   - Enter email, password, name
   - Click "Sign Up"
   - ✅ Should auto-login and redirect to homepage
   - ✅ Check localStorage for `authToken`

2. **Login**:
   - Navigate to http://localhost:3000/login
   - Enter valid credentials
   - Click "Log In"
   - ✅ Should redirect to homepage or intended page
   - ✅ User name should appear in navbar

3. **Protected Routes**:
   - Without login, navigate to http://localhost:3000/profile
   - ✅ Should redirect to `/login?redirect=/profile`
   - After login, verify redirect back to /profile

4. **Logout**:
   - Click "Logout" in navbar
   - ✅ Should clear session and redirect to homepage
   - ✅ `authToken` removed from localStorage

#### Language Detection

1. **English Query**:
   - Open chatbot (bottom-right icon)
   - Type: "What is a humanoid robot?"
   - ✅ Response in English

2. **Urdu Script Query**:
   - Type: "روبوٹکس کیا ہے؟"
   - ✅ Response in Urdu (Arabic script)

3. **Roman Urdu Query**:
   - Type: "Robotics kya hai?"
   - ✅ Response in Urdu (detection accuracy 85%+)

4. **Mixed Query**:
   - Type: "Humanoid robot kaise kaam karta hai?"
   - ✅ Should detect as Urdu if 30%+ Urdu keywords

#### UI Enhancements

1. **Typography**:
   - Open any book chapter
   - ✅ Body text should be 17px, line-height 1.7
   - ✅ Headings should have proper hierarchy

2. **Homepage Hero**:
   - Visit http://localhost:3000
   - ✅ Hero section should have gradient background
   - ✅ Title and subtitle should be centered and styled

3. **Spacing**:
   - ✅ Sections should have 2.5rem top margin
   - ✅ Cards should have 2rem padding
   - ✅ Navigation items should have 0.5rem spacing

4. **Responsive**:
   - Open DevTools and test mobile view (375px width)
   - ✅ Font sizes should adjust
   - ✅ Layout should be mobile-friendly

---

## Troubleshooting

### Frontend Issues

#### "Cannot find module '@docusaurus/core'"

```bash
rm -rf node_modules package-lock.json
npm install
```

#### "REACT_APP_API_URL is undefined"

- Ensure `.env.local` exists in `humanoid-robotics-book/` directory
- Restart dev server (`npm start`)
- Docusaurus requires `REACT_APP_` prefix for custom variables

#### Auth not working (401 errors)

- Check backend is running at http://localhost:8000
- Verify `REACT_APP_API_URL` points to correct backend
- Check browser console for CORS errors

### Backend Issues

#### "OPENAI_API_KEY not found"

- Ensure `.env` file exists in `backend/` directory
- Check `.env` has `OPENAI_API_KEY=sk-...`
- Restart backend server

#### "Qdrant connection failed"

- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Test connection: `curl https://your-cluster.qdrant.tech:6333/collections`
- Check Qdrant dashboard for cluster status

#### "No module named 'database'"

```bash
# Activate virtual environment first
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

#### Chat returns "No context found"

- Qdrant collection may be empty
- Run embedding population script:
  ```bash
  python main.py
  ```
- Verify collection exists: Check Qdrant dashboard

---

## Deployment

### Frontend (Vercel)

1. **Connect Repository**:
   - Visit https://vercel.com/new
   - Import Git repository
   - Select `humanoid-robotics-book` directory as root

2. **Configure Build**:
   - Framework Preset: Docusaurus
   - Build Command: `npm run build`
   - Output Directory: `build`
   - Install Command: `npm install`

3. **Environment Variables**:
   ```
   REACT_APP_API_URL=https://hackathon1-production-aaf0.up.railway.app
   ```

4. **Deploy**:
   - Click "Deploy"
   - Wait for build to complete
   - Visit deployment URL

### Backend (Railway)

1. **Create New Project**:
   - Visit https://railway.app/new
   - Connect GitHub repository
   - Select `backend` directory

2. **Configure Service**:
   - Language: Python
   - Start Command: `uvicorn main_fastapi:app --host 0.0.0.0 --port $PORT`

3. **Environment Variables**:
   ```
   DATABASE_URL=sqlite:///./app.db
   BETTER_AUTH_SECRET=<32-char-secret>
   BETTER_AUTH_URL=https://<your-railway-domain>.railway.app
   OPENAI_API_KEY=<your-key>
   QDRANT_URL=<your-qdrant-url>
   QDRANT_API_KEY=<your-key>
   QDRANT_COLLECTION_NAME=humanoid_robotics_docs
   ALLOWED_ORIGINS=https://<your-vercel-domain>.vercel.app
   ```

4. **Deploy**:
   - Push to main branch
   - Railway auto-deploys
   - Check logs for startup errors

---

## Performance Optimization

### Frontend

- **Code Splitting**: Docusaurus automatically splits routes
- **Image Optimization**: Use WebP format, compress to <500KB
- **Font Loading**: System fonts (no web font overhead)
- **CSS Minification**: Enabled in production build
- **Bundle Size**: Target <200KB gzipped

### Backend

- **Session Storage**: Migrate to Redis from in-memory dict (production)
- **Qdrant Caching**: Cache frequent queries (future enhancement)
- **OpenAI Rate Limits**: Implement request throttling if needed
- **Database Indexing**: Already indexed on `users.email` and `chat_history.user_id`

---

## Next Steps

After local setup is complete:

1. **Implement feature** following `/sp.tasks` generated tasks
2. **Test thoroughly** using manual checklist above
3. **Run type check**: `npm run typecheck`
4. **Build locally**: `npm run build` (verify no errors)
5. **Create PR** with feature branch → main
6. **Deploy to staging** (Vercel preview deployment)
7. **QA testing** with real Urdu speakers
8. **Deploy to production** (merge to main)

---

## Resources

- **Docusaurus Docs**: https://docusaurus.io/docs
- **Better Auth Docs**: https://better-auth.com/docs
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **OpenAI API Docs**: https://platform.openai.com/docs
- **Qdrant Docs**: https://qdrant.tech/documentation/

---

## Support

For issues or questions:
- GitHub Issues: [Repository Issues Page]
- Backend logs: Check Railway deployment logs
- Frontend logs: Check Vercel deployment logs
- Local debugging: Browser DevTools Console + Network tab

**Happy coding!** 🚀
