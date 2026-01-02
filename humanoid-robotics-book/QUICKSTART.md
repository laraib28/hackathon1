# Quick Start Guide

Get your Humanoid Robotics Book running in 5 minutes!

## Prerequisites

- Node.js v18+
- PostgreSQL v14+
- OpenAI API Key ([Get one here](https://platform.openai.com/api-keys))

---

## Step 1: Install Dependencies

```bash
# Install frontend dependencies
npm install

# Install server dependencies
cd server
npm install
cd ..
```

---

## Step 2: Configure Environment

### Update `server/.env`:

```env
# Required
DATABASE_URL=postgresql://your_username:your_password@localhost:5432/humanoid_robotics_db
OPENAI_API_KEY=sk-your-openai-key-here
BETTER_AUTH_SECRET=your-random-32-character-secret-key-here

# Already configured (keep as is)
PORT=3001
CLIENT_URL=http://localhost:3000
BETTER_AUTH_URL=http://localhost:3001
```

### Verify `.env.local` (should exist):

```env
REACT_APP_API_URL=http://localhost:3001
```

---

## Step 3: Setup Database

```bash
# Create PostgreSQL database
sudo -u postgres psql -c "CREATE DATABASE humanoid_robotics_db;"

# Create user (if needed)
sudo -u postgres psql -c "CREATE USER your_username WITH PASSWORD 'your_password';"
sudo -u postgres psql -c "GRANT ALL PRIVILEGES ON DATABASE humanoid_robotics_db TO your_username;"

# Run migrations
cd server
npm run migrate
```

Expected output:
```
✅ Database migrations completed successfully!
📊 Tables created:
   - users
   - accounts
   - sessions
   - verification_tokens
   - user_preferences
```

---

## Step 4: Start Servers

Open **two terminal windows**:

### Terminal 1 - Auth Server:
```bash
cd server
npm run dev
```

Wait for:
```
🚀 Better Auth server running on http://localhost:3001
```

### Terminal 2 - Frontend:
```bash
npm start
```

Wait for:
```
[SUCCESS] Serving "humanoid-robotics-book" at http://localhost:3000
```

---

## Step 5: Test It Out!

1. **Open browser:** http://localhost:3000
2. **Sign up:** http://localhost:3000/signup
   - Create account with email/password
3. **Test chatbot:** Click chat icon (bottom-right corner)
   - Ask a question in English or Urdu
4. **Manage preferences:** http://localhost:3000/profile
   - Change language, theme, privacy settings

---

## Troubleshooting

### "Server is not responding" in chatbot
```bash
# Check if server is running
curl http://localhost:3001/api/health

# If not, start it
cd server && npm run dev
```

### Database connection errors
```bash
# Check PostgreSQL is running
sudo systemctl status postgresql  # Linux
brew services list | grep postgresql  # macOS

# Test database connection
psql -U your_username -d humanoid_robotics_db
```

### Migration fails
```bash
# Recreate database
sudo -u postgres psql
DROP DATABASE IF EXISTS humanoid_robotics_db;
CREATE DATABASE humanoid_robotics_db;
GRANT ALL PRIVILEGES ON DATABASE humanoid_robotics_db TO your_username;
\q

# Run migration again
cd server && npm run migrate
```

---

## What's Available Now

✅ **Authentication**
- Email/password signup/login
- Google OAuth (if configured)
- Session management

✅ **User Preferences**
- Language (English/Urdu)
- Theme (Light/Dark/System)
- Chat history toggle
- Notifications toggle

✅ **AI Chatbot**
- OpenAI-powered responses
- Bilingual (English/Urdu)
- Context-aware
- Chat history

✅ **Profile Management**
- View user info
- Update preferences
- Logout

---

## Important URLs

- **Home:** http://localhost:3000
- **Login:** http://localhost:3000/login
- **Signup:** http://localhost:3000/signup
- **Profile:** http://localhost:3000/profile
- **Health Check:** http://localhost:3001/api/health

---

## Next Steps

- Read [SETUP_GUIDE.md](./SETUP_GUIDE.md) for detailed documentation
- Read [CHANGES_SUMMARY.md](./CHANGES_SUMMARY.md) for feature overview
- Configure Google/GitHub OAuth (optional)
- Customize chatbot prompts in `server/index.ts`

---

**Need help?** Check the [Troubleshooting section in SETUP_GUIDE.md](./SETUP_GUIDE.md#troubleshooting)

Happy coding! 🚀
