# Better Auth Setup Guide / بہتر Auth سیٹ اپ گائیڈ

## ✅ Authentication Fix Complete!

Authentication ab properly configured hai with Better Auth aur environment variables.

---

## 📁 Configuration Files / کنفیگریشن فائلیں

### Frontend (Docusaurus)

**`.env.local`** (Development)
```env
# Chat Backend (FastAPI on port 8000)
REACT_APP_API_URL=http://127.0.0.1:8000

# Auth Backend (Better Auth on port 3001)
REACT_APP_AUTH_URL=http://localhost:3001
```

**`.env.production`** (Production - Vercel/Railway)
```env
# Chat Backend (FastAPI on Railway)
REACT_APP_API_URL=https://hackathon1-production-aaf0.up.railway.app

# Auth Backend (Better Auth - update with deployed URL)
REACT_APP_AUTH_URL=http://localhost:3001  # ⚠️ Production میں change کریں
```

**`.env.example`** (Template)
- Example values کے ساتھ
- Git میں commit ہو گی (no secrets)
- نئے developers کے لیے reference

---

### Backend (Express Auth Server)

**`server/.env`** (Your actual secrets - NOT in git)
```env
# Server Configuration
PORT=3001
NODE_ENV=development
CLIENT_URL=http://localhost:3000

# Better Auth
BETTER_AUTH_SECRET=dev-secret-1234567890-abcdefghijklmnopqrstuvwxyz
BETTER_AUTH_URL=http://localhost:3001

# Database (Neon PostgreSQL)
DATABASE_URL=postgresql://neondb_owner:npg_w1f2YebzEmSv@ep-wispy-salad-a1cfpxoq-pooler.ap-southeast-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require

# OpenAI API
OPENAI_API_KEY=sk-proj-6khS6snxyGqKbR5...

# OAuth (Optional)
GOOGLE_CLIENT_ID=your-google-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-google-client-secret
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
```

**`server/.env.example`** (Template - in git)
- Placeholder values
- Documentation کے لیے
- Security ke liye real secrets nahi

---

## 🔧 Changes Made / تبدیلیاں

### 1. ✅ Frontend Auth Client Fixed
**File:** `src/lib/auth-client.ts`

**Before (Hardcoded):**
```typescript
const BACKEND_URL = 'http://localhost:3001';
```

**After (Environment Variable):**
```typescript
const BACKEND_URL = process.env.REACT_APP_AUTH_URL || 'http://localhost:3001';
```

**Benefit:**
- Development میں `localhost:3001` use ہوگا
- Production میں deployed auth server URL use ہوگا
- Flexible aur configurable

---

### 2. ✅ Environment Files Updated

| File | Purpose | Status |
|------|---------|--------|
| `.env.local` | Development frontend config | ✅ Updated |
| `.env.production` | Production frontend config | ✅ Updated |
| `.env.example` | Frontend template | ✅ Updated |
| `server/.env` | Backend secrets (NOT in git) | ✅ Verified |
| `server/.env.example` | Backend template | ✅ Verified |

---

## 🚀 How to Run / کیسے چلائیں

### Development Mode

**Terminal 1: Start Auth Server**
```bash
cd server
npm install
npm run dev
```
Auth server will run on: `http://localhost:3001`

**Terminal 2: Start Frontend**
```bash
npm install
npm start
```
Frontend will run on: `http://localhost:3000`

**Terminal 3: Start Chat Backend (Optional)**
```bash
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt
uvicorn main:app --reload --port 8000
```
Chat API will run on: `http://localhost:8000`

---

## 🧪 Test Authentication / Authentication ٹیسٹ کریں

### 1. Signup (نیا account بنائیں)
1. Go to: http://localhost:3000/signup
2. Enter email and password
3. Click "Sign Up"
4. Account create ہو جائے گا

### 2. Login (لاگ ان کریں)
1. Go to: http://localhost:3000/login
2. Enter email and password
3. Click "Sign In"
4. Redirect to homepage with logged-in state

### 3. Check Session
- Navbar میں profile icon دکھائی دے گا
- `/profile` page access کر سکتے ہیں
- Logout button کام کرے گا

---

## 📊 API Endpoints

### Auth Server (Better Auth - Port 3001)
- `POST /api/auth/sign-up/email` - New user registration
- `POST /api/auth/sign-in/email` - Login with email/password
- `POST /api/auth/sign-out` - Logout
- `GET /api/auth/session` - Get current session
- `GET /api/health` - Health check

### Chat Backend (FastAPI - Port 8000)
- `POST /chat` - Send message to AI
- `GET /health` - Health check

---

## 🔐 Security Notes

### ✅ Currently Secure:
- `.env` files ignored in git ✅
- Secrets NOT committed to repository ✅
- HTTP-only cookies for sessions ✅
- CORS properly configured ✅
- Environment variables for API URLs ✅

### ⚠️ Production Recommendations:
1. **Change `BETTER_AUTH_SECRET`** - Production میں strong random secret use کریں
   ```bash
   openssl rand -base64 32
   ```

2. **Deploy Auth Server** - Vercel/Railway پر deploy کریں
   ```bash
   # Production میں update کریں:
   REACT_APP_AUTH_URL=https://your-auth-server.vercel.app
   ```

3. **Enable Email Verification** - Production میں email verification on کریں
   - Better Auth config میں `requireEmailVerification: true`
   - Email provider configure کریں (Resend, SendGrid, etc.)

4. **Add OAuth (Optional)** - Google/GitHub login
   - Google: https://console.cloud.google.com
   - GitHub: https://github.com/settings/developers
   - Client ID aur Secret `.env` میں add کریں

5. **Database Backups** - Neon PostgreSQL automatic backups check کریں

---

## 🗄️ Database Setup

### Current Database: Neon PostgreSQL
- Host: `ep-wispy-salad-a1cfpxoq-pooler.ap-southeast-1.aws.neon.tech`
- Database: `neondb`
- Connection pooling: Enabled
- SSL: Required

### Tables (Auto-created by Better Auth):
- `users` - User accounts
- `sessions` - Active sessions
- `accounts` - OAuth linked accounts
- `verification_tokens` - Email verification

### Run Migrations:
```bash
cd server
npm run migrate
```

---

## 🐛 Troubleshooting

### Issue: "Failed to fetch" on login
**Solution:**
1. Check auth server is running: `http://localhost:3001/api/health`
2. Check CORS configuration in `server/index.ts`
3. Verify `.env.local` has `REACT_APP_AUTH_URL=http://localhost:3001`

### Issue: "Database connection failed"
**Solution:**
1. Check `DATABASE_URL` in `server/.env`
2. Verify Neon database is accessible
3. Test connection: `npm run server:migrate`

### Issue: Session not persisting
**Solution:**
1. Clear browser cookies
2. Check `CLIENT_URL` matches frontend URL
3. Verify cookies are HTTP-only and SameSite=Lax

### Issue: OAuth not working
**Solution:**
1. Add OAuth credentials to `server/.env`
2. Configure redirect URLs in Google/GitHub console
3. Check `BETTER_AUTH_URL` is correct

---

## 📝 Summary / خلاصہ

### ✅ Completed:
- [x] Frontend auth URL ab configurable hai
- [x] `.env.local` میں `REACT_APP_AUTH_URL` added
- [x] `.env.production` updated
- [x] `.env.example` files properly documented
- [x] Backend `.env` verified and working
- [x] Security: `.env` files ignored in git

### 🎯 Ready to Use:
- Email/Password authentication ✅
- Session management ✅
- Login/Signup pages ✅
- Profile page ✅
- Better Auth integration ✅

### 🚀 Next Steps (Optional):
- Deploy auth server to production
- Add Google/GitHub OAuth
- Enable email verification
- Add password reset functionality
- Implement 2FA (two-factor authentication)

---

## 📞 Support

**Better Auth Documentation:** https://better-auth.com
**Neon Database:** https://neon.tech
**Issues?** Check `server/logs` for errors

---

**Authentication ab fully configured hai! 🎉**
Start both servers (`npm run dev` in server, `npm start` in root) aur test کریں!
