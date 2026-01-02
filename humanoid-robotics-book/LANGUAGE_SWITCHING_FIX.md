# Language Switching Fix Guide

## Problem
When clicking the Urdu language switcher in development mode, you get a "Page Not Found" error.

## Root Cause
**Docusaurus dev server (`npm start`) only serves the default locale (English) for performance.** Urdu routes are NOT generated until you run a production build.

When you click the language switcher, it tries to navigate to `/ur/intro/` or similar paths, but these routes don't exist in development mode.

## Solutions

### ✅ Solution 1: Use Production Build (Recommended)
This enables full multi-locale support with working language switching:

```bash
# Build and serve production version
npm run start:prod

# Or manually:
npm run build
npm run serve
```

**Access:**
- English: http://localhost:3000/
- Urdu: http://localhost:3000/ur/

The language switcher will work correctly in this mode.

### ✅ Solution 2: Test Single Locale in Dev Mode
If you only want to test Urdu without building:

```bash
# Run dev server with Urdu only
npm run start:ur
```

**Note:** In this mode, the site will show Urdu content, but the language switcher won't work because only Urdu locale is loaded.

### ✅ Solution 3: Standard Dev Mode (English Only)
For regular development:

```bash
npm start
```

**Note:** Language switcher won't work - only English content is available.

## Summary

| Command | Locales Available | Language Switcher | Use Case |
|---------|-------------------|-------------------|----------|
| `npm start` | English only | ❌ No | Regular development |
| `npm run start:ur` | Urdu only | ❌ No | Test Urdu in dev mode |
| `npm run start:prod` | English + Urdu | ✅ Yes | Test language switching |

## Why This Limitation Exists
Docusaurus disables multi-locale in dev mode for:
- Faster startup time
- Lower memory usage
- Quicker hot-reload performance

Production builds generate all locale routes, enabling full language switching.

## Current Status
✅ i18n configuration is correct
✅ Urdu translation files exist (33 docs)
✅ RTL support is configured
✅ Navbar translations are complete
✅ Production build generates Urdu routes

The issue is **NOT a bug** - it's expected Docusaurus behavior in development mode.
