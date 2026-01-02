# Better Auth Server Setup

This directory contains the Better Auth authentication server for the Humanoid Robotics Book application.

## Prerequisites

- Node.js 20+
- PostgreSQL database
- (Optional) Google OAuth credentials
- (Optional) GitHub OAuth credentials

## Quick Start

### 1. Install Dependencies

```bash
cd server
npm install
```

### 2. Set Up Environment Variables

Copy the example environment file:

```bash
cp .env.example .env
```

Edit `.env` and configure:

**Required:**
- `DATABASE_URL` - Your PostgreSQL connection string
- `BETTER_AUTH_SECRET` - A random secret key (min 32 characters)

**Optional (for OAuth):**
- `GOOGLE_CLIENT_ID` & `GOOGLE_CLIENT_SECRET`
- `GITHUB_CLIENT_ID` & `GITHUB_CLIENT_SECRET`

### 3. Set Up Database

Run the migration to create necessary tables:

```bash
npm run migrate
```

This will create the following tables:
- `users` - User accounts
- `accounts` - OAuth provider accounts
- `sessions` - User sessions
- `verification_tokens` - Email verification tokens

### 4. Start the Server

**Development:**
```bash
npm run dev
```

**Production:**
```bash
npm run build
npm start
```

The server will run on `http://localhost:3001`

## Environment Variables

### Required

| Variable | Description | Example |
|----------|-------------|---------|
| `DATABASE_URL` | PostgreSQL connection string | `postgresql://user:pass@localhost:5432/dbname` |
| `BETTER_AUTH_SECRET` | Secret key for auth (min 32 chars) | `your-very-long-secret-key-here` |

### Optional

| Variable | Description | Default |
|----------|-------------|---------|
| `PORT` | Server port | `3001` |
| `NODE_ENV` | Environment | `development` |
| `CLIENT_URL` | Frontend URL for CORS | `http://localhost:3000` |
| `BETTER_AUTH_URL` | Auth server URL | `http://localhost:3001` |
| `GOOGLE_CLIENT_ID` | Google OAuth client ID | - |
| `GOOGLE_CLIENT_SECRET` | Google OAuth secret | - |
| `GITHUB_CLIENT_ID` | GitHub OAuth client ID | - |
| `GITHUB_CLIENT_SECRET` | GitHub OAuth secret | - |

## Getting OAuth Credentials

### Google OAuth

1. Go to [Google Cloud Console](https://console.cloud.google.com)
2. Create a new project or select existing one
3. Enable Google+ API
4. Go to Credentials → Create Credentials → OAuth client ID
5. Set authorized redirect URI: `http://localhost:3001/api/auth/callback/google`
6. Copy Client ID and Client Secret to `.env`

### GitHub OAuth

1. Go to [GitHub Developer Settings](https://github.com/settings/developers)
2. Click "New OAuth App"
3. Set Authorization callback URL: `http://localhost:3001/api/auth/callback/github`
4. Copy Client ID and Client Secret to `.env`

## API Endpoints

The server provides the following Better Auth endpoints:

- `POST /api/auth/sign-up/email` - Email/password signup
- `POST /api/auth/sign-in/email` - Email/password login
- `GET /api/auth/sign-in/google` - Google OAuth login
- `GET /api/auth/sign-in/github` - GitHub OAuth login
- `POST /api/auth/sign-out` - Sign out
- `GET /api/auth/session` - Get current session
- `GET /api/health` - Health check

## Deployment

### Railway

1. Install Railway CLI: `npm install -g @railway/cli`
2. Login: `railway login`
3. Initialize: `railway init`
4. Add PostgreSQL: `railway add`
5. Set environment variables in Railway dashboard
6. Deploy: `railway up`

### Vercel (with Vercel Postgres)

1. Install Vercel CLI: `npm install -g vercel`
2. Deploy: `vercel`
3. Add Vercel Postgres from dashboard
4. Set environment variables in Vercel dashboard

## Troubleshooting

### Database Connection Issues

- Verify `DATABASE_URL` is correct
- Ensure PostgreSQL is running
- Check firewall/network settings
- For Railway/Vercel, ensure database add-on is properly linked

### OAuth Issues

- Verify redirect URIs match exactly (including http/https)
- Ensure OAuth apps are not in testing mode
- Check that CLIENT_ID and CLIENT_SECRET are correct

### CORS Issues

- Verify `CLIENT_URL` matches your frontend URL
- Check that credentials are included in frontend requests
- Ensure cookies are allowed in browser

## Development

### Project Structure

```
server/
├── auth.ts          # Better Auth configuration
├── index.ts         # Express server
├── migrate.ts       # Database migrations
├── package.json     # Dependencies
├── tsconfig.json    # TypeScript config
└── .env            # Environment variables
```

### Adding New Features

Better Auth is highly extensible. See [Better Auth Docs](https://better-auth.com) for:
- Adding more OAuth providers
- Email verification
- Password reset
- Two-factor authentication
- Rate limiting
- Custom plugins

## Support

For issues or questions:
- Better Auth Docs: https://better-auth.com
- GitHub Issues: https://github.com/laraib28/hackathon1/issues
