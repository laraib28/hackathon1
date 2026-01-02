# Humanoid Robotics and Physical AI Guide

An interactive educational platform for learning about humanoid robotics and physical AI, built with [Docusaurus](https://docusaurus.io/).

## Features

✨ **Better Auth Integration**
- Email/password authentication
- Google & GitHub OAuth support
- Secure session management

🤖 **AI-Powered Chatbot**
- OpenAI GPT integration
- Bilingual support (English/Urdu)
- Context-aware responses

👤 **User Personalization**
- Language preferences
- Theme selection (Light/Dark/System)
- Privacy controls
- Chat history management

📚 **Educational Content**
- Comprehensive robotics documentation
- Urdu translations
- Interactive learning experience

---

## Quick Start

**New to this project?** Start here: **[QUICKSTART.md](./QUICKSTART.md)**

For detailed setup instructions: **[SETUP_GUIDE.md](./SETUP_GUIDE.md)**

For a summary of recent changes: **[CHANGES_SUMMARY.md](./CHANGES_SUMMARY.md)**

---

## Installation

```bash
# Frontend dependencies
npm install

# Server dependencies
cd server
npm install
```

## Local Development

**You need to run TWO servers:**

### Terminal 1 - Auth Server
```bash
cd server
npm run dev
```

### Terminal 2 - Frontend
```bash
npm start
```

The frontend will open at http://localhost:3000 and the auth server runs at http://localhost:3001.

## Environment Configuration

### Frontend (`.env.local`)
```env
REACT_APP_API_URL=http://localhost:3001
```

### Server (`server/.env`)
```env
DATABASE_URL=postgresql://username:password@localhost:5432/humanoid_robotics_db
OPENAI_API_KEY=sk-your-openai-key-here
BETTER_AUTH_SECRET=your-secret-key-min-32-chars
# ... see server/.env.example for complete configuration
```

## Database Setup

```bash
# Create database
sudo -u postgres psql -c "CREATE DATABASE humanoid_robotics_db;"

# Run migrations
cd server
npm run migrate
```

## Build

```bash
# Frontend build
npm run build

# Server build
cd server
npm run build
```

## Project Structure

```
humanoid-robotics-book/
├── src/                          # Frontend source
│   ├── components/               # React components
│   │   ├── Auth/                # Authentication components
│   │   ├── ChatWidget/          # AI chatbot widget
│   │   └── UserPreferences/     # User settings UI
│   ├── context/                 # React context (auth, etc.)
│   ├── lib/                     # Libraries and utilities
│   └── pages/                   # Docusaurus pages
├── server/                       # Backend server
│   ├── auth.ts                  # Better Auth configuration
│   ├── index.ts                 # Express server + API endpoints
│   └── migrate.ts               # Database migrations
├── docs/                         # Documentation content
├── static/                       # Static assets
├── .env.local                   # Frontend environment vars
└── SETUP_GUIDE.md               # Comprehensive setup guide
```

## Documentation

- **[QUICKSTART.md](./QUICKSTART.md)** - Get started in 5 minutes
- **[SETUP_GUIDE.md](./SETUP_GUIDE.md)** - Detailed setup and configuration
- **[CHANGES_SUMMARY.md](./CHANGES_SUMMARY.md)** - Recent updates and features

## Tech Stack

- **Frontend:** React, Docusaurus, TypeScript
- **Backend:** Express.js, Better Auth, TypeScript
- **Database:** PostgreSQL
- **AI:** OpenAI GPT API
- **Authentication:** Better Auth (email/password, OAuth)

## Deployment

See [SETUP_GUIDE.md](./SETUP_GUIDE.md) for production deployment instructions.

For GitHub Pages deployment:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

## Support

Need help? Check:
1. [Troubleshooting section](./SETUP_GUIDE.md#troubleshooting) in SETUP_GUIDE.md
2. Server logs for detailed errors
3. Browser console for frontend errors

## License

This project is licensed under the MIT License.
