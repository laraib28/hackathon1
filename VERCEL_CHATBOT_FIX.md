# Vercel Chatbot Deployment Fix

## Problem
The chatbot was not appearing on the Vercel deployed site because it was trying to connect to `http://localhost:8000/api/chat` which doesn't exist in the production environment.

## Root Cause
The `ChatWidget.tsx` component had a hardcoded URL pointing to the local development backend instead of using environment variables for different environments.

## Solution Implemented

### 1. Updated ChatWidget.tsx
- Changed hardcoded URL: `'http://localhost:8000/api/chat'`
- To environment variable: `process.env.REACT_APP_API_URL || 'http://localhost:8000'`
- Now uses production backend URL in deployed environments

### 2. Configured Environment Variables
- **Development** (`humanoid-robotics-book/.env.local`):
  ```
  REACT_APP_API_URL=http://localhost:8000
  ```
- **Production** (`humanoid-robotics-book/.env.production`):
  ```
  REACT_APP_API_URL=https://hackathon1-production-aaf0.up.railway.app
  ```

### 3. Updated Vercel Environment
- Added `REACT_APP_API_URL` environment variable in Vercel project settings
- Points to the deployed backend: `https://hackathon1-production-aaf0.up.railway.app`

## Files Modified
- `humanoid-robotics-book/src/components/ChatWidget/ChatWidget.tsx`
- `humanoid-robotics-book/.env.production`

## How to Deploy
1. Make sure your backend is deployed and accessible (e.g., on Railway)
2. Add environment variable in Vercel dashboard:
   - Key: `REACT_APP_API_URL`
   - Value: Your backend URL (e.g., `https://your-backend.onrender.com`)
3. Redeploy the frontend on Vercel

## Verification Steps
1. Deploy to Vercel
2. Open the deployed site
3. Click the chatbot button
4. The chatbot should appear and be able to communicate with the backend
5. Open browser developer tools to check for any network errors

## Troubleshooting
- If chatbot still doesn't work, check browser console for CORS errors
- Verify that the backend URL is accessible and responding to requests
- Ensure the `REACT_APP_API_URL` environment variable is set correctly in Vercel

## Backend Health Check
To verify the backend is working, test this endpoint:
```
curl https://hackathon1-production-aaf0.up.railway.app/health
```
Should return a healthy status response.