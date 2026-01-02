# Better Auth Setup Guide

This guide will help you set up Better Auth authentication for the Humanoid Robotics Book application.

## What Was Changed

The authentication system has been migrated to use [Better Auth](https://better-auth.com), a modern authentication solution with:

- ✅ Email/Password authentication
- ✅ Google OAuth
- ✅ GitHub OAuth
- ✅ PostgreSQL database
- ✅ Secure session management
- ✅ Built-in CSRF protection

## Architecture

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│                 │         │                 │         │                 │
│  Docusaurus     │────────▶│  Auth Server    │────────▶│   PostgreSQL    │
│  Frontend       │         │  (Express)      │         │   Database      │
│                 │         │                 │         │                 │
└─────────────────┘         └─────────────────┘         └─────────────────┘
     Port 3000                   Port 3001
```

## Setup Steps

### Step 1: PostgreSQL Database

You need a PostgreSQL database. Choose one option:

#### Option A: Local PostgreSQL

```bash
# Install PostgreSQL (macOS)
brew install postgresql@15
brew services start postgresql@15

# Create database
createdb better_auth_db
```

#### Option B: Railway (Recommended for Production)

1. Go to [Railway](https://railway.app)
2. Create new project
3. Add PostgreSQL database
4. Copy the `DATABASE_URL` connection string

#### Option C: Vercel Postgres

1. Install Vercel CLI: `npm install -g vercel`
2. Run: `vercel`
3. Add Postgres from Vercel dashboard
4. Copy connection string

### Step 2: Configure Backend Server

```bash
cd server

# Copy example environment file
cp .env.example .env

# Edit .env with your values
nano .env
```

**Required configurations in `server/.env`:**

```env
# Database (REQUIRED)
DATABASE_URL=postgresql://username:password@localhost:5432/better_auth_db

# Secret key (REQUIRED - generate a random 32+ character string)
BETTER_AUTH_SECRET=your-super-secret-key-min-32-characters-long

# Server settings
PORT=3001
NODE_ENV=development
CLIENT_URL=http://localhost:3000
BETTER_AUTH_URL=http://localhost:3001
```

### Step 3: Set Up OAuth Providers (Optional)

#### Google OAuth

1. Visit [Google Cloud Console](https://console.cloud.google.com)
2. Create/select a project
3. Enable "Google+ API"
4. Go to **Credentials** → **Create Credentials** → **OAuth client ID**
5. Application type: **Web application**
6. Authorized redirect URIs:
   - Development: `http://localhost:3001/api/auth/callback/google`
   - Production: `https://your-domain.com/api/auth/callback/google`
7. Copy **Client ID** and **Client Secret** to `server/.env`:

```env
GOOGLE_CLIENT_ID=your-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-client-secret
```

#### GitHub OAuth

1. Visit [GitHub Developer Settings](https://github.com/settings/developers)
2. Click **New OAuth App**
3. Fill in:
   - Application name: `Humanoid Robotics Book`
   - Homepage URL: `http://localhost:3000`
   - Authorization callback URL: `http://localhost:3001/api/auth/callback/github`
4. Click **Register application**
5. Copy **Client ID** and generate **Client Secret**
6. Add to `server/.env`:

```env
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
```

### Step 4: Run Database Migration

```bash
cd server
npm run migrate
```

Expected output:
```
🔄 Running database migrations...
✅ Database migrations completed successfully!
📊 Tables created:
   - users
   - accounts
   - sessions
   - verification_tokens
```

### Step 5: Start the Server

**Development:**
```bash
# Terminal 1 - Start auth server
cd server
npm run dev

# Terminal 2 - Start Docusaurus
npm start
```

**Production:**
```bash
cd server
npm run build
npm start
```

### Step 6: Verify Setup

1. Backend health check:
   - Open: http://localhost:3001/api/health
   - Should see: `{"status":"ok","message":"Better Auth server is running"}`

2. Frontend:
   - Open: http://localhost:3000
   - Try signing up with email/password
   - Try Google/GitHub OAuth (if configured)

## Project Structure

```
humanoid-robotics-book/
├── server/                    # Better Auth backend
│   ├── auth.ts               # Auth configuration
│   ├── index.ts              # Express server
│   ├── migrate.ts            # Database setup
│   ├── package.json          # Server dependencies
│   ├── .env                  # Server environment variables
│   └── README.md             # Server documentation
│
├── src/
│   ├── lib/
│   │   └── auth-client.ts    # Better Auth client
│   ├── services/
│   │   └── authService.ts    # Auth service wrapper
│   ├── context/
│   │   └── AuthContext.tsx   # Auth context provider
│   └── components/
│       └── Auth/
│           ├── Login.tsx     # Login component
│           └── Signup.tsx    # Signup component
│
├── .env.local                # Frontend environment variables
└── .env.production           # Production environment variables
```

## Environment Variables

### Frontend (`.env.local`)

```env
REACT_APP_API_URL=http://localhost:3001
```

### Backend (`server/.env`)

See `server/.env.example` for all available options.

## Deployment

### Railway Deployment

1. **Deploy Backend:**
   ```bash
   cd server
   # Push to git repository
   git add .
   git commit -m "Add Better Auth server"
   git push

   # Railway will auto-detect and deploy
   ```

2. **Set Environment Variables in Railway:**
   - Go to Railway dashboard
   - Select your project
   - Go to Variables tab
   - Add all variables from `server/.env.example`
   - Railway will provide `DATABASE_URL` automatically

3. **Update Frontend:**
   - Edit `.env.production`:
     ```env
     REACT_APP_API_URL=https://your-railway-app.railway.app
     ```
   - Update OAuth redirect URIs to use production URL

### Vercel Deployment

1. Deploy backend separately (Railway recommended)
2. Deploy frontend to Vercel
3. Set `REACT_APP_API_URL` in Vercel environment variables

## Troubleshooting

### "Database connection failed"

- Verify `DATABASE_URL` in `server/.env`
- Ensure PostgreSQL is running
- Check database credentials
- For Railway: ensure database is linked to project

### "CORS error"

- Verify `CLIENT_URL` in `server/.env` matches frontend URL
- Check that `credentials: true` is set in CORS config
- Ensure cookies are enabled in browser

### "OAuth redirect mismatch"

- Verify redirect URIs in Google/GitHub OAuth settings
- Must exactly match: `http://localhost:3001/api/auth/callback/google`
- Use `http://` for localhost, `https://` for production

### "Session not persisting"

- Check browser cookie settings
- Verify `BETTER_AUTH_SECRET` is set
- Ensure backend and frontend URLs are correct
- Check browser console for errors

## Testing

### Test Email/Password Auth

```bash
# Sign up
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"password123","name":"Test User"}'

# Sign in
curl -X POST http://localhost:3001/api/auth/sign-in/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"password123"}'
```

### Test Session

```bash
# Get current session (include cookies from sign-in)
curl http://localhost:3001/api/auth/session \
  --cookie "better-auth.session=your-session-token"
```

## Next Steps

1. ✅ Set up PostgreSQL database
2. ✅ Configure environment variables
3. ✅ Run database migration
4. ✅ Start backend server
5. ✅ Test authentication flow
6. Optional: Set up Google OAuth
7. Optional: Set up GitHub OAuth
8. Deploy to production

## Support

- **Better Auth Docs:** https://better-auth.com
- **PostgreSQL Docs:** https://www.postgresql.org/docs/
- **Railway Docs:** https://docs.railway.app
- **Project Issues:** https://github.com/laraib28/hackathon1/issues

## Security Notes

- ⚠️ Never commit `.env` files to git
- ⚠️ Use strong random values for `BETTER_AUTH_SECRET` (32+ characters)
- ⚠️ Always use HTTPS in production
- ⚠️ Keep OAuth credentials secure
- ✅ Better Auth handles password hashing automatically
- ✅ Sessions are cryptographically signed
- ✅ CSRF protection is built-in
