# ChatWidget Backend Connection Setup Guide

This guide explains how to connect your Docusaurus ChatWidget with the FastAPI backend.

## Quick Start

### 1. Backend Setup (Port 8000)

```bash
cd backend

# Activate virtual environment
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Start the FastAPI server
python main_fastapi.py

# OR using uvicorn directly
uvicorn main_fastapi:app --reload --host 0.0.0.0 --port 8000
```

Verify backend is running:
- Open: http://localhost:8000
- Should see: "Welcome to Humanoid Robotics Search API"
- Test chat endpoint: http://localhost:8000/api/chat (POST)

### 2. Frontend Setup (Port 3000)

```bash
cd humanoid-robotics-book

# Install dependencies (if needed)
npm install

# Start development server
npm start
```

Frontend will run at: http://localhost:3000

## Configuration

### Environment Variables

**`.env.local`** (for development):
```bash
REACT_APP_API_URL=http://localhost:8000
```

**`.env.production`** (for production):
```bash
REACT_APP_API_URL=https://your-backend-url.com
```

### How It Works

1. **ChatWidget.tsx** reads `process.env.REACT_APP_API_URL`
2. Falls back to `http://localhost:8000` if not set
3. Constructs full endpoint: `${API_BASE_URL}/api/chat`
4. Makes POST request with message and language

## API Endpoints

### POST /api/chat

**Request:**
```json
{
  "message": "What is ROS2?",
  "target_language": "en",
  "selected_text": null,
  "conversation_history": [],
  "user_id": null
}
```

**Response:**
```json
{
  "response": "ROS2 (Robot Operating System 2) is...",
  "sources": [
    {
      "url": "/docs/...",
      "score": 0.85,
      "content_preview": "..."
    }
  ],
  "language": "en"
}
```

## Troubleshooting

### Issue: 404 Error - Cannot POST /chat

**Problem:** Request going to `/chat` instead of `/api/chat`

**Solution:**
✅ Already fixed! ChatWidget now uses `/api/chat` correctly.

Verify in browser console:
```
🔄 Sending request to: http://localhost:8000/api/chat
📡 Response status: 200
✅ Response received: {...}
```

### Issue: CORS Error

**Problem:**
```
Access to fetch at 'http://localhost:8000/api/chat' from origin 'http://localhost:3000'
has been blocked by CORS policy
```

**Solution:**
Backend already has CORS configured in `main_fastapi.py`:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

If still having issues:
1. Restart backend server
2. Clear browser cache
3. Try incognito mode

### Issue: Network Error / Connection Refused

**Problem:** `Fetch error: TypeError: Failed to fetch`

**Checklist:**
- [ ] Backend running on port 8000?
  ```bash
  curl http://localhost:8000/health
  ```
- [ ] Correct URL in `.env.local`?
- [ ] No firewall blocking port 8000?
- [ ] Backend crashed? Check terminal for errors

**Debug:**
```bash
# Check if port 8000 is in use
lsof -i :8000  # Mac/Linux
netstat -ano | findstr :8000  # Windows

# Test backend directly
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "test", "target_language": "en"}'
```

### Issue: Backend Returns 500 Error

**Problem:** Backend error in processing request

**Check:**
1. OpenAI API key configured?
   ```bash
   # In backend/.env
   OPENAI_API_KEY=sk-...
   ```

2. Qdrant configured?
   ```bash
   QDRANT_URL=https://...
   QDRANT_API_KEY=...
   ```

3. Embeddings populated?
   ```bash
   cd backend
   python check_embeddings.py
   # Should show: Total vectors: 1012
   ```

4. Check backend logs in terminal

### Issue: Empty or Generic Responses

**Problem:** Bot responds but with generic answers

**Cause:** Qdrant search not finding relevant documents

**Solution:**
1. Verify embeddings exist:
   ```bash
   cd backend
   python check_embeddings.py
   ```

2. If no embeddings (0 vectors):
   ```bash
   python populate_embeddings.py
   ```

3. Test search:
   ```bash
   python search.py
   # Try: "what is ROS2?"
   ```

4. Adjust score threshold in `chat_api.py` (line 88):
   ```python
   score_threshold=0.6  # Lower = more lenient (try 0.3-0.5)
   ```

## Development Workflow

### Running Both Services Together

**Terminal 1 - Backend:**
```bash
cd backend
source .venv/bin/activate
python main_fastapi.py
```

**Terminal 2 - Frontend:**
```bash
cd humanoid-robotics-book
npm start
```

**Terminal 3 - Testing:**
```bash
# Test backend
curl http://localhost:8000/health

# Test frontend
curl http://localhost:3000
```

## Production Deployment

### Backend (e.g., Railway, Render, Fly.io)

1. Deploy FastAPI app
2. Set environment variables:
   ```
   OPENAI_API_KEY=...
   QDRANT_URL=...
   QDRANT_API_KEY=...
   DATABASE_URL=...
   ```
3. Note the production URL (e.g., `https://my-api.railway.app`)

### Frontend (Vercel)

1. Set environment variable in Vercel:
   ```
   REACT_APP_API_URL=https://my-api.railway.app
   ```

2. Redeploy frontend

3. Test the chat widget on production site

## Browser Console Debugging

Open browser DevTools (F12) and check Console tab:

**Successful Request:**
```
🔄 Sending request to: http://localhost:8000/api/chat
📡 Response status: 200
✅ Response received: {response: "...", sources: [...], language: "en"}
```

**Failed Request:**
```
🔄 Sending request to: http://localhost:8000/api/chat
📡 Response status: 404
❌ Backend error: 404 Not Found
```

**Network Request Details:**
1. Open Network tab
2. Send a message in chat
3. Click on the request
4. Check:
   - Request URL: Should be `http://localhost:8000/api/chat`
   - Request Method: POST
   - Status Code: 200 OK
   - Response: JSON with response and sources

## Files Modified

| File | Purpose |
|------|---------|
| `src/components/ChatWidget/ChatWidget.tsx` | Updated fetch URL to use environment variable |
| `.env.local` | Development API URL configuration |
| `.env.production` | Production API URL configuration |
| `docusaurus.config.ts` | Added customFields for API configuration |
| `backend/main_fastapi.py` | CORS configuration (already correct) |
| `backend/chat_api.py` | Chat endpoint (already correct) |

## Testing Checklist

- [ ] Backend running on http://localhost:8000
- [ ] Backend `/health` endpoint returns healthy status
- [ ] Backend `/api/chat` endpoint accepts POST requests
- [ ] Frontend running on http://localhost:3000
- [ ] ChatWidget opens when clicking button
- [ ] Sending message shows loading indicator
- [ ] Response appears in chat (not error message)
- [ ] Browser console shows successful requests (200 status)
- [ ] No CORS errors in browser console

## Advanced: Custom Proxy (Optional)

If you need a proxy for other reasons, create `docusaurus-proxy-plugin.js`:

```javascript
module.exports = function (context, options) {
  return {
    name: 'docusaurus-proxy-plugin',
    configureWebpack(config, isServer) {
      if (!isServer) {
        return {
          devServer: {
            proxy: {
              '/api': {
                target: 'http://localhost:8000',
                changeOrigin: true,
                secure: false,
              },
            },
          },
        };
      }
      return {};
    },
  };
};
```

Then add to `docusaurus.config.ts`:
```typescript
plugins: ['./docusaurus-proxy-plugin.js'],
```

With proxy, you can use relative URLs:
```typescript
fetch('/api/chat', { ... })  // Proxied to http://localhost:8000/api/chat
```

## Support

If issues persist:
1. Check browser console for errors
2. Check backend terminal for errors
3. Test backend endpoint directly with curl
4. Verify all environment variables are set
5. Restart both frontend and backend servers
