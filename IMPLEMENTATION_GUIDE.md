# Authentication and Urdu Language Implementation Guide

## Overview
This guide provides a complete implementation of user authentication with login/signup protection and Urdu language support using Docusaurus built-in i18n.

## Files Changed/Added

### 1. Authentication Protection
- `src/theme/Root.tsx` - Main entry point wrapped with AuthProvider and ProtectedRoute
- `src/components/ProtectedRoute.tsx` - Component to protect routes and redirect unauthenticated users
- `src/components/Auth/Login.tsx` - Updated to use useNavigate and handle i18n redirects
- `src/components/Auth/Signup.tsx` - Updated to use useNavigate and handle i18n redirects
- `src/lib/auth-client.ts` - Updated with proper backend URL logic for different environments

### 2. Urdu Language Support
- `docusaurus.config.ts` - Updated with localeDropdown and Urdu RTL support
- `i18n/ur/` - Complete folder structure with translations
- `src/css/rtl.css` - Already existed with RTL styling

## Step-by-Step Instructions

### Step 1: Environment Setup
1. Ensure you have the backend running on port 8000
2. Set environment variables in `.env.local`:
   ```
   REACT_APP_API_URL=http://localhost:8000
   ```

### Step 2: Verify Authentication Protection
1. When visiting book content pages (like `/part1-foundations/intro`) without being logged in:
   - You will be redirected to `/login`
   - The redirect URL will be preserved in the query parameter
2. After successful login, you'll be redirected back to the original page
3. Public pages like homepage (`/`), login (`/login`), and signup (`/signup`) remain accessible

### Step 3: Verify Urdu Language Support
1. Language switcher should appear in the navbar (using `type: 'localeDropdown'`)
2. Switching between English and Urdu:
   - English pages: `/part1-foundations/intro`
   - Urdu pages: `/ur/part1-foundations/intro`
3. If an Urdu translation doesn't exist, it will fallback to English content automatically
4. RTL styling is applied when viewing Urdu content

### Step 4: Verify i18n-Aware Authentication
1. When trying to access `/ur/part1-foundations/intro` without authentication:
   - You'll be redirected to `/ur/login`
   - After login, you'll be redirected back to `/ur/part1-foundations/intro`
2. The authentication components now properly handle i18n redirects
3. Links between login/signup pages maintain locale context

### Step 5: Adding More Urdu Translations
To add more Urdu translations:
1. Copy the English content from `docs/` to `i18n/ur/docusaurus-plugin-content-docs/current/`
2. Translate the content while keeping the same file structure
3. Update frontmatter (title, sidebar_label, etc.) to Urdu equivalents

Example translation:
```markdown
---
title: ہیومنوڈ روبوٹکس کا تعارف  # Urdu title
sidebar_label: تعارف  # Urdu sidebar label
slug: /
description: ہیومنوڈ روبوٹکس اور فزیکل AI کے بارے میں سیکھیں  # Urdu description
---

# ہیومنوڈ روبوٹکس کا تعارف  # Urdu heading

(Translated content here...)
```

### Step 6: Testing the Implementation
1. Start the development server:
   ```bash
   cd humanoid-robotics-book
   npm run start
   ```

2. Test authentication:
   - Visit a book page without logging in - should redirect to login
   - Sign up for a new account
   - Log in and verify you can access book pages
   - Log out and verify book pages are protected again

3. Test language switching:
   - Switch between English and Urdu using the navbar dropdown
   - Verify RTL styling is applied to Urdu content
   - Verify that translated pages show Urdu text
   - Verify that untranslated pages fallback to English

4. Test combined functionality:
   - In Urdu mode, try to access a book page without authentication
   - Should redirect to Urdu login page
   - After login, should return to the Urdu book page

## Important Notes
- The authentication system uses Better Auth with the existing backend setup
- All book routes starting with `/part1-foundations/`, `/part2-modules/`, `/part3-capstone/`, `/part4-future/` and their Urdu counterparts (`/ur/...`) are protected
- Homepage, login, and signup pages are public and accessible without authentication
- The system properly handles redirects while maintaining locale context
- RTL styling is automatically applied when viewing Urdu content
- Translation fallback mechanism works - if Urdu translation doesn't exist, English version is shown
- Chat widget functionality is preserved and works with the authentication system

## Troubleshooting
If login redirects don't work properly:
- Check that backend is running on the configured port
- Verify API endpoints are accessible
- Check browser console for any authentication errors

If language switching doesn't work:
- Clear browser cache
- Verify `docusaurus.config.ts` has correct locale configuration
- Ensure `i18n/ur/` directory has proper structure with content