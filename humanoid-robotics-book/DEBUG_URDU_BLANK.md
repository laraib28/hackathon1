# Debug Urdu Blank Page Issue

## Problem
Urdu پر click کرنے سے blank page آ رہا ہے۔

## Debugging Steps

### Step 1: Browser Console Check
1. Browser میں F12 دبائیں (DevTools open کرنے کے لیے)
2. **Console** tab پر جائیں
3. Urdu language پر click کریں
4. Console میں **red errors** دیکھیں

**Common Errors to Look For:**
- `Cannot read property of undefined`
- `useDocusaurusContext`
- `BrowserOnly`
- `Loading chunk failed`
- `404` or network errors

### Step 2: Network Tab Check
1. DevTools میں **Network** tab کھولیں
2. Urdu پر click کریں
3. دیکھیں کہ کون سی files fail ho رہی ہیں (red status)

### Step 3: Check Current URL
جب blank page آئے تو URL دیکھیں:
- ✅ Should be: `http://localhost:3000/ur/` or `http://localhost:3000/ur/intro/`
- ❌ If different, there's a routing issue

### Step 4: Verify Server Running
```bash
# Check which port is serving
lsof -i :3000
# یا
netstat -an | grep 3000
```

## Quick Fixes to Try

### Fix 1: Clear Browser Cache
```
Ctrl+Shift+Delete (Windows/Linux)
Cmd+Shift+Delete (Mac)
```
- Clear "Cached images and files"
- Reload page

### Fix 2: Rebuild
```bash
# Stop current server (Ctrl+C)
npm run build
npm run serve
```

### Fix 3: Clear Docusaurus Cache
```bash
npm run clear
npm run build
npm run serve
```

### Fix 4: Hard Refresh
- Press: **Ctrl+Shift+R** (Windows/Linux)
- Or: **Cmd+Shift+R** (Mac)

## Common Causes

### 1. BrowserOnly Issue
**Symptom:** Blank page, no errors
**Cause:** Client-side only components not wrapped properly
**Fix:** Check `Root.tsx` and auth components

### 2. Build Cache
**Symptom:** Old build serving
**Cause:** Stale `.docusaurus` cache
**Fix:** `npm run clear && npm run build`

### 3. JavaScript Error
**Symptom:** Blank white page
**Cause:** Uncaught exception in render
**Fix:** Check console, fix the error

### 4. Missing Translations
**Symptom:** Blank or partial page
**Cause:** Missing Urdu content files
**Fix:** Verify `i18n/ur/` has all docs

## What to Report

Please send me:
1. **Console errors** (screenshot or copy text)
2. **Current URL** when blank page shows
3. **Command you're running** (npm start, serve, etc.)
4. **Network tab** - any red/failed requests

## Quick Test Commands

```bash
# Test 1: Is build directory ok?
ls -la build/ur/ | head

# Test 2: Is server running?
curl http://localhost:3000/ur/

# Test 3: Check for JavaScript errors in build
grep -r "error" build/ur/*.js | head -5
```

---

**Please try Step 1 (Browser Console) first and tell me what errors you see!**
