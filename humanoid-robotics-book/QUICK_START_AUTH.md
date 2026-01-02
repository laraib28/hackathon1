# Quick Start - Better Auth Setup

Get authentication running in 5 minutes!

## Prerequisites

- ✅ Node.js 20+ installed
- ✅ PostgreSQL database (local or cloud)

## Option 1: Automated Setup (Recommended)

### Step 1: Run the setup script

```bash
chmod +x start-dev.sh
./start-dev.sh
```

This will:
- Create `.env` files if missing
- Install all dependencies
- Start both auth server (port 3001) and frontend (port 3000)

### Step 2: Configure Database

Edit `server/.env` and add your PostgreSQL connection:

```env
DATABASE_URL=postgresql://username:password@localhost:5432/database_name
```

### Step 3: Run Migration

```bash
npm run server:migrate
```

### Step 4: Restart servers

Press `Ctrl+C` and run `./start-dev.sh` again.

✅ **Done!** Visit http://localhost:3000 and try signing up!

---

## Option 2: Manual Setup

### Step 1: Install Dependencies

```bash
# Frontend
npm install

# Backend
cd server
npm install
cd ..
```

### Step 2: Configure Environment

```bash
# Create server environment file
cp server/.env.example server/.env

# Create frontend environment file
cp .env.example .env.local
```

### Step 3: Edit `server/.env`

**Required - Update these:**

```env
DATABASE_URL=postgresql://username:password@localhost:5432/better_auth_db
BETTER_AUTH_SECRET=your-random-32-character-secret-key-here
```

**Optional - For OAuth:**

```env
GOOGLE_CLIENT_ID=your-google-client-id
GOOGLE_CLIENT_SECRET=your-google-client-secret
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
```

### Step 4: Create Database Tables

```bash
npm run server:migrate
```

Expected output:
```
✅ Database migrations completed successfully!
📊 Tables created:
   - users
   - accounts
   - sessions
   - verification_tokens
```

### Step 5: Start Development Servers

**Terminal 1 - Auth Server:**
```bash
npm run server:dev
```

**Terminal 2 - Frontend:**
```bash
npm start
```

### Step 6: Test It Out

1. Open http://localhost:3000
2. Click "Sign Up"
3. Enter email and password
4. Click "Create Account"

✅ You should be redirected and logged in!

---

## Troubleshooting

### "Database connection failed"

**Check:**
- ✅ PostgreSQL is running: `pg_isready`
- ✅ Database exists: `psql -l | grep better_auth`
- ✅ Connection string is correct in `server/.env`

**Fix for local PostgreSQL:**
```bash
# Create database if it doesn't exist
createdb better_auth_db

# Test connection
psql better_auth_db -c "SELECT version();"
```

### "Migration failed"

**Solution:**
```bash
# Drop and recreate database (WARNING: deletes all data)
dropdb better_auth_db
createdb better_auth_db

# Run migration again
npm run server:migrate
```

### "OAuth redirect mismatch"

**Fix:**
1. Go to Google/GitHub OAuth settings
2. Add redirect URI: `http://localhost:3001/api/auth/callback/google`
3. Must be exact match (including `http://` and port)

### "CORS error"

**Check `server/.env`:**
```env
CLIENT_URL=http://localhost:3000
```

Must match your frontend URL exactly.

### "Port already in use"

**Solution:**
```bash
# Check what's using port 3001
lsof -ti:3001

# Kill the process
kill -9 $(lsof -ti:3001)

# Or use different port in server/.env
PORT=3002
```

---

## Quick Reference

### Useful Commands

```bash
# Start both servers
./start-dev.sh

# Start auth server only
npm run server:dev

# Start frontend only
npm start

# Run database migration
npm run server:migrate

# Check server health
curl http://localhost:3001/api/health

# Test signup
curl -X POST http://localhost:3001/api/auth/sign-up/email \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"test123","name":"Test User"}'
```

### URLs

- **Frontend:** http://localhost:3000
- **Auth Server:** http://localhost:3001
- **Health Check:** http://localhost:3001/api/health
- **API Endpoints:** http://localhost:3001/api/auth/*

### Environment Files

- `server/.env` - Backend configuration (database, secrets, OAuth)
- `.env.local` - Frontend configuration (backend URL)
- `.env.production` - Production frontend config

---

## Next Steps

1. ✅ Test email/password authentication
2. ✅ Set up Google OAuth (optional)
3. ✅ Set up GitHub OAuth (optional)
4. ✅ Deploy to production

For detailed deployment instructions, see [BETTER_AUTH_SETUP.md](./BETTER_AUTH_SETUP.md)

---

## Need Help?

- **Detailed Guide:** [BETTER_AUTH_SETUP.md](./BETTER_AUTH_SETUP.md)
- **Server Docs:** [server/README.md](./server/README.md)
- **Better Auth Docs:** https://better-auth.com
- **Report Issues:** https://github.com/laraib28/hackathon1/issues
