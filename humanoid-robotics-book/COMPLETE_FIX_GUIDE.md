# Complete Fix Guide - Chatbot & Urdu Pages

## Problem Summary

You reported two issues:
1. **Chatbot not working** - "Sorry, the server is not responding"
2. **Urdu pages not working** - Translation incomplete

---

## Issue #1: Chatbot Not Working ❌

### Root Cause
Your application requires **THREE** services to run, but you're probably only running TWO:

| Service | Port | Purpose | Currently Running? |
|---------|------|---------|-------------------|
| Frontend (Docusaurus) | 3000 | Main website | ✅ Likely YES |
| Auth Server (Better Auth) | 3001 | User authentication | ✅ Likely YES |
| **ChatBackend (FastAPI)** | **8000** | **Chatbot & RAG** | ❌ **Probably NO** |

The chatbot needs the FastAPI backend on port 8000, which is NOT started by `start-dev.sh`.

### ✅ Solution

You need to run **3 terminals** (or use the new script below):

#### Option A: Manual Start (3 Terminals)

**Terminal 1 - Frontend:**
```bash
npm start
```

**Terminal 2 - Auth Server:**
```bash
cd server
npm run dev
```

**Terminal 3 - Chat Backend (MISSING!):**
```bash
cd ../backend
python3 main_fastapi.py
```

#### Option B: Use the New All-in-One Script (Recommended)

I've created a new script that starts ALL THREE services:

```bash
chmod +x start-all-services.sh
./start-all-services.sh
```

This will start:
- Auth Server on port 3001
- Frontend on port 3000
- **Chat Backend on port 8000** ← This was missing!

### Verify It's Working

1. **Check Backend Health:**
   ```bash
   curl http://localhost:8000/health
   ```
   Should return: `{"status": "healthy", ...}`

2. **Check Chat Endpoint:**
   ```bash
   curl -X POST http://localhost:8000/api/chat \
     -H "Content-Type: application/json" \
     -d '{"message":"test","target_language":"en"}'
   ```

3. **Test Chatbot in Browser:**
   - Open http://localhost:3000
   - Click the chat button (💬)
   - Send a message
   - You should get a response!

---

## Issue #2: Urdu Pages Not Working ❌

### Root Cause
The Urdu translation is **only 4% complete**. The infrastructure is set up correctly, but most content is missing.

### Current Status

✅ **What Works:**
- Urdu locale configured in Docusaurus
- Language switcher in navbar
- RTL (right-to-left) support
- Directory structure created
- 4-5 chapters partially translated

❌ **What Doesn't Work:**
- Most chapters (20+ chapters) are untranslated
- Some pages may be empty or English-only
- Translation is incomplete

### ✅ Temporary Solution

**Option 1: Hide Urdu Until Complete**

Edit `docusaurus.config.ts` line 29:
```typescript
// Before:
locales: ['en', 'ur'],

// After (temporarily disable Urdu):
locales: ['en'],  // Removed 'ur' until translation is complete
```

**Option 2: Add "Translation in Progress" Notice**

Keep Urdu enabled but add a banner informing users that translation is incomplete. This requires creating a custom component.

**Option 3: Complete the Translation**

The translation needs approximately **40-60 hours** of work by an Urdu technical translator.

See `URDU_TRANSLATION_STATUS.md` for details on what needs to be translated.

### Verify Language Switcher

1. Open http://localhost:3000
2. Look for language dropdown in top-right (should show "English" and "اردو")
3. Click on "اردو"
4. URL should change to `/ur/`
5. You'll see partial content (incomplete translations)

---

## Quick Checklist

### For Chatbot to Work:
- [ ] Auth server running on port 3001
- [ ] Frontend running on port 3000
- [ ] **FastAPI backend running on port 8000** ← KEY!
- [ ] `.env.local` has `REACT_APP_API_URL=http://localhost:8000`
- [ ] Backend `.env` has valid `OPENAI_API_KEY`

### For Urdu Pages:
- [ ] Understand only 4% is translated
- [ ] Decide: hide Urdu, show notice, or complete translation
- [ ] Locale switcher works (infrastructure is fine)

---

## Updated Architecture Diagram

```
┌─────────────────────────────────────────────────┐
│                    Browser                       │
│              http://localhost:3000               │
└───────────────────┬─────────────────────────────┘
                    │
        ┌───────────┴───────────┬──────────────────┐
        │                       │                  │
        ▼                       ▼                  ▼
┌──────────────┐      ┌──────────────┐   ┌──────────────┐
│  Frontend    │      │ Auth Server  │   │ Chat Backend │
│  Docusaurus  │      │ Better Auth  │   │   FastAPI    │
│              │      │              │   │              │
│  Port 3000   │      │  Port 3001   │   │  Port 8000   │
│              │      │              │   │              │
│ - Displays   │      │ - Login      │   │ - Chatbot    │
│   content    │      │ - Signup     │   │ - RAG search │
│ - Language   │      │ - Sessions   │   │ - OpenAI     │
│   switcher   │      │              │   │ - Qdrant     │
└──────────────┘      └──────────────┘   └──────────────┘
```

---

## Files Modified in This Fix

1. ✅ `.env.local` - Updated API URL from port 3001 → 8000
2. ✅ `.env.example` - Documented correct port
3. ✅ `start-all-services.sh` - NEW script to start all 3 services
4. ✅ `COMPLETE_FIX_GUIDE.md` - This guide

---

## Next Steps

1. **Start all 3 services** using `start-all-services.sh`
2. **Test chatbot** - should work now!
3. **Decide on Urdu pages** - hide, notice, or translate
4. **Verify** everything works before committing

---

## Need More Help?

- Chatbot backend code: `../backend/main_fastapi.py`
- Chatbot API: `../backend/chat_api.py`
- Urdu status: `URDU_TRANSLATION_STATUS.md`
- Auth setup: `QUICK_START_AUTH.md`
- Server setup: `server/README.md`

**Report issues:** https://github.com/laraib28/hackathon1/issues
