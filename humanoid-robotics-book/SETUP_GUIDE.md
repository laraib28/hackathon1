# Humanoid Robotics Book - Complete Setup Guide

This guide will help you set up and run the Humanoid Robotics Book project with Better Auth authentication and AI-powered chatbot.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Project Architecture](#project-architecture)
3. [Environment Setup](#environment-setup)
4. [Database Setup](#database-setup)
5. [Running the Application](#running-the-application)
6. [Testing the Features](#testing-the-features)
7. [Troubleshooting](#troubleshooting)
8. [Features Overview](#features-overview)

---

## Prerequisites

Before you begin, ensure you have the following installed:

- **Node.js** (v18 or higher) - [Download](https://nodejs.org/)
- **npm** or **yarn** - Comes with Node.js
- **PostgreSQL** (v14 or higher) - [Download](https://www.postgresql.org/download/)
- **Git** - [Download](https://git-scm.com/)

---

## Project Architecture

This project consists of three main components:

1. **Frontend (Docusaurus)** - Port 3000
   - React-based documentation site
   - Better Auth client integration
   - AI chatbot widget
   - User preferences UI

2. **Auth Server (Express + Better Auth)** - Port 3001
   - Better Auth authentication endpoints
   - User preferences API
   - OpenAI-powered chat API

3. **Database (PostgreSQL)** - Port 5432
   - User accounts and sessions
   - User preferences
   - OAuth provider data

---

## Environment Setup

### 1. Clone the Repository

```bash
git clone <your-repo-url>
cd humanoid-robotics-book
```

### 2. Install Dependencies

#### Frontend Dependencies

```bash
npm install
```

#### Server Dependencies

```bash
cd server
npm install
cd ..
```

### 3. Configure Environment Variables

#### Frontend Configuration

Create or update `.env.local` in the project root:

```env
# Backend API URL for Better Auth and Chat
REACT_APP_API_URL=http://localhost:3001
```

#### Server Configuration

Update `server/.env` with your credentials:

```env
# Server Configuration
PORT=3001
NODE_ENV=development

# Client URL (for CORS)
CLIENT_URL=http://localhost:3000

# Better Auth Configuration
BETTER_AUTH_SECRET=your-super-secret-key-change-this-in-production-min-32-chars
BETTER_AUTH_URL=http://localhost:3001

# PostgreSQL Database
DATABASE_URL=postgresql://your_username:your_password@localhost:5432/humanoid_robotics_db

# OpenAI API Key (Required for chatbot - get from https://platform.openai.com/api-keys)
OPENAI_API_KEY=sk-your-openai-api-key-here

# Google OAuth (Optional - get from https://console.cloud.google.com)
GOOGLE_CLIENT_ID=your-google-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-google-client-secret

# GitHub OAuth (Optional - get from https://github.com/settings/developers)
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
```

**Important:** Replace the placeholder values with your actual credentials:
- Generate a secure random string for `BETTER_AUTH_SECRET` (minimum 32 characters)
- Get OpenAI API key from [OpenAI Platform](https://platform.openai.com/api-keys)
- Configure OAuth providers if you want Google/GitHub login

---

## Database Setup

### 1. Install PostgreSQL

If you haven't installed PostgreSQL yet:

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install postgresql postgresql-contrib
```

**macOS:**
```bash
brew install postgresql@14
brew services start postgresql@14
```

**Windows:**
Download and install from [postgresql.org](https://www.postgresql.org/download/windows/)

### 2. Create Database and User

```bash
# Access PostgreSQL prompt
sudo -u postgres psql

# Create database
CREATE DATABASE humanoid_robotics_db;

# Create user
CREATE USER your_username WITH PASSWORD 'your_password';

# Grant privileges
GRANT ALL PRIVILEGES ON DATABASE humanoid_robotics_db TO your_username;

# Exit PostgreSQL
\q
```

### 3. Update DATABASE_URL

Update `server/.env` with your actual database credentials:

```env
DATABASE_URL=postgresql://your_username:your_password@localhost:5432/humanoid_robotics_db
```

### 4. Run Database Migrations

```bash
cd server
npm run migrate
```

You should see:
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

## Running the Application

You'll need **three terminal windows** to run all components.

### Terminal 1: Auth Server

```bash
cd server
npm run dev
```

You should see:
```
🚀 Better Auth server running on http://localhost:3001
✅ Health check: http://localhost:3001/api/health
```

### Terminal 2: Frontend (Docusaurus)

```bash
# From project root
npm start
```

You should see:
```
[SUCCESS] Serving "humanoid-robotics-book" at http://localhost:3000
```

### Terminal 3: (Optional) Database Monitor

```bash
# Monitor database logs if needed
sudo -u postgres tail -f /var/log/postgresql/postgresql-14-main.log
```

---

## Testing the Features

### 1. Test Health Endpoints

**Check Auth Server:**
```bash
curl http://localhost:3001/api/health
```

Expected response:
```json
{"status":"ok","message":"Better Auth server is running"}
```

### 2. Test Authentication

1. Open your browser to [http://localhost:3000](http://localhost:3000)
2. Click "Login" or navigate to [http://localhost:3000/login](http://localhost:3000/login)
3. Click "Sign up instead" or go to [http://localhost:3000/signup](http://localhost:3000/signup)
4. Create a new account:
   - Full Name: Test User
   - Email: test@example.com
   - Password: password123 (minimum 6 characters)
5. Click "Sign Up"
6. You should be redirected to the home page, logged in

### 3. Test Profile and Preferences

1. Navigate to [http://localhost:3000/profile](http://localhost:3000/profile)
2. You should see:
   - Your user information (name, email, user ID)
   - Language preference (English/Urdu)
   - Theme preference (Light/Dark/System)
   - Privacy settings (chat history, notifications)
3. Change some preferences and click "Save Preferences"
4. Refresh the page - your preferences should persist

### 4. Test Chatbot

1. Look for the chat widget in the bottom-right corner of any page
2. Click the chat icon to open it
3. Type a message in English or Urdu
4. You should receive an AI-generated response

**If chatbot shows "server is not responding":**
- Check that OpenAI API key is set in `server/.env`
- Verify the auth server is running on port 3001
- Check browser console for errors

### 5. Test OAuth (Optional)

If you configured Google OAuth:

1. Go to [http://localhost:3000/login](http://localhost:3000/login)
2. Click "Sign in with Google"
3. Complete Google authentication
4. You should be logged in

---

## Troubleshooting

### Problem: "Server is not responding" in chatbot

**Cause:** Backend server is not running or OPENAI_API_KEY is not configured

**Solution:**
1. Check if server is running: `curl http://localhost:3001/api/health`
2. If not running, start it: `cd server && npm run dev`
3. Verify OPENAI_API_KEY is set in `server/.env`
4. Restart the server after adding the key

### Problem: "Connection refused" on port 3001

**Cause:** Auth server is not running

**Solution:**
```bash
cd server
npm run dev
```

### Problem: Database connection errors

**Cause:** PostgreSQL is not running or credentials are incorrect

**Solution:**
1. Check PostgreSQL is running:
   ```bash
   sudo systemctl status postgresql  # Linux
   brew services list | grep postgresql  # macOS
   ```
2. Verify DATABASE_URL in `server/.env`
3. Test connection:
   ```bash
   psql -U your_username -d humanoid_robotics_db -h localhost
   ```

### Problem: Migration fails

**Cause:** Database doesn't exist or user doesn't have permissions

**Solution:**
1. Recreate the database:
   ```sql
   DROP DATABASE IF EXISTS humanoid_robotics_db;
   CREATE DATABASE humanoid_robotics_db;
   GRANT ALL PRIVILEGES ON DATABASE humanoid_robotics_db TO your_username;
   ```
2. Run migration again: `npm run migrate`

### Problem: Frontend can't reach backend

**Cause:** CORS or wrong API URL

**Solution:**
1. Verify `.env.local` has `REACT_APP_API_URL=http://localhost:3001`
2. Restart frontend: `npm start`
3. Check browser console for CORS errors
4. Verify `CLIENT_URL` in `server/.env` is `http://localhost:3000`

### Problem: OAuth redirect errors

**Cause:** Incorrect OAuth credentials or redirect URIs

**Solution:**
1. For Google OAuth, set authorized redirect URIs:
   - `http://localhost:3001/api/auth/callback/google`
2. For GitHub OAuth, set callback URL:
   - `http://localhost:3001/api/auth/callback/github`
3. Verify client IDs and secrets in `server/.env`

---

## Features Overview

### ✅ Better Auth Integration

- **Email/Password Authentication**
  - Secure password hashing
  - Session management (7-day expiry, 1-day refresh)
  - CSRF protection

- **OAuth Providers**
  - Google Sign-In (optional)
  - GitHub Sign-In (optional)

- **Protected Routes**
  - Automatic redirection for unauthenticated users
  - Post-login redirect to original page

### ✅ User Personalization

- **User Preferences**
  - Language selection (English/Urdu)
  - Theme selection (Light/Dark/System)
  - Privacy controls (chat history, notifications)
  - Persisted to PostgreSQL database

- **Profile Management**
  - View user information
  - Update preferences
  - Logout functionality

### ✅ AI-Powered Chatbot

- **Features**
  - OpenAI GPT integration
  - Bilingual support (English/Urdu)
  - Context-aware responses
  - Text selection for contextual queries
  - Chat history persistence (if enabled)

- **Fallback Responses**
  - Works even without OpenAI key (limited functionality)
  - Intelligent keyword matching
  - User-friendly error messages

### ✅ Database Schema

**Tables:**
- `users` - User accounts
- `accounts` - OAuth provider data
- `sessions` - Active sessions
- `verification_tokens` - Email verification
- `user_preferences` - User settings and preferences

---

## Quick Reference

### Useful Commands

```bash
# Start everything (requires 2 terminals)
cd server && npm run dev  # Terminal 1
npm start                 # Terminal 2 (from project root)

# Database operations
cd server
npm run migrate           # Run migrations
psql -U your_username -d humanoid_robotics_db  # Access database

# Check logs
tail -f server/logs/app.log  # Server logs (if configured)

# Build for production
npm run build
cd server && npm run build
```

### Important URLs

- **Frontend:** http://localhost:3000
- **Auth Server:** http://localhost:3001
- **Health Check:** http://localhost:3001/api/health
- **Login:** http://localhost:3000/login
- **Signup:** http://localhost:3000/signup
- **Profile:** http://localhost:3000/profile

### Environment Files

- `.env.local` - Frontend configuration
- `server/.env` - Backend configuration
- `.env.example` - Example configuration (frontend)
- `server/.env.example` - Example configuration (server)

---

## Security Notes

1. **Never commit `.env` files** to version control
2. Use strong, unique secrets in production
3. Enable email verification in production (`requireEmailVerification: true`)
4. Use SSL/TLS for database connections in production
5. Set `NODE_ENV=production` in production
6. Use environment-specific OAuth callback URLs

---

## Next Steps

1. **Customize the chatbot** by updating system prompts in `server/index.ts`
2. **Add more OAuth providers** (Twitter, Facebook, etc.) in `server/auth.ts`
3. **Enhance user preferences** by adding more fields to `user_preferences` table
4. **Set up email verification** for production use
5. **Deploy to production** (Vercel, Railway, Render, etc.)

---

## Support

If you encounter issues:

1. Check the [Troubleshooting](#troubleshooting) section
2. Review server logs for detailed error messages
3. Check browser console for frontend errors
4. Verify all environment variables are set correctly

For Better Auth documentation: [https://better-auth.com](https://better-auth.com)
For OpenAI API: [https://platform.openai.com/docs](https://platform.openai.com/docs)

---

**Happy coding!** 🚀
