# Language Switching Test Guide

## Issue
Language dropdown appears but clicking doesn't switch pages.

## Root Cause
Docusaurus dev server (`npm start`) only runs **default locale (English)** for performance. Multi-locale requires production build.

## Solution: Build & Test

### Step 1: Build Site
```bash
cd humanoid-robotics-book
npm run build
```

This creates:
- `/build/` - English pages
- `/build/ur/` - Urdu pages

### Step 2: Serve Locally
```bash
npm run serve
```

Opens at: `http://localhost:3000`

### Step 3: Test Language Switching

#### Test 1: Switch to Urdu
1. Go to `http://localhost:3000`
2. Click language dropdown (top-right)
3. Select **اردو**
4. ✅ URL changes to `http://localhost:3000/ur/`
5. ✅ Content switches to Urdu
6. ✅ Page direction becomes RTL (right-to-left)

#### Test 2: Switch to English
1. From Urdu page `http://localhost:3000/ur/`
2. Click language dropdown
3. Select **English**
4. ✅ URL changes to `http://localhost:3000/`
5. ✅ Content switches to English
6. ✅ Page direction becomes LTR (left-to-right)

#### Test 3: Direct URL Access
- English: `http://localhost:3000/intro`
- Urdu: `http://localhost:3000/ur/intro`
- Both should load respective language versions

### Step 4: Verify Translations

Check these pages have proper translations:
- [ ] `/` → `/ur/` (Homepage)
- [ ] `/intro` → `/ur/intro` (Introduction)
- [ ] `/part1-foundations/intro` → `/ur/part1-foundations/intro`
- [ ] Navbar labels (📚 Book → 📚 کتاب)
- [ ] Footer links (Foundations → بنیادی باتیں)

## For Development with Live Reload

If you need hot-reload during translation work:

```bash
# Option 1: English only (fast)
npm start

# Option 2: Include Urdu (slower, but both languages work)
npm start -- --locale ur
```

## Production Deployment

When deployed to Vercel/Netlify:
1. Build command: `npm run build`
2. Both locales included automatically
3. Language switcher works out-of-the-box

## Troubleshooting

### Language dropdown visible but doesn't switch
- **Cause:** Running `npm start` without `--locale ur`
- **Fix:** Use `npm run build && npm run serve` instead

### Some pages missing in Urdu
- **Cause:** Translation files not created yet
- **Fix:** Docusaurus shows English as fallback
- Create translation: Copy from `/docs/` to `/i18n/ur/docusaurus-plugin-content-docs/current/`

### RTL styling broken
- **Cause:** Custom CSS not applied
- **Fix:** Check `/src/css/rtl.css` and `Root.tsx` locale detection

## URL Structure Reference

### English
- Homepage: `/`
- Pages: `/intro`, `/part1-foundations/intro`, etc.

### Urdu
- Homepage: `/ur/`
- Pages: `/ur/intro`, `/ur/part1-foundations/intro`, etc.

## Current Status

✅ i18n configured in `docusaurus.config.ts`
✅ Locale dropdown in navbar
✅ 30+ Urdu pages translated
✅ RTL support for Urdu
✅ Navbar/footer translated to Urdu
✅ Automatic fallback to English

**Next Step:** Run `npm run build && npm run serve` to test!
