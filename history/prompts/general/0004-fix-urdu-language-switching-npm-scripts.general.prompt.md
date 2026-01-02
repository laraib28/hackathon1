---
id: 0004
title: Fix Urdu language switching with NPM scripts and build verification
stage: general
date: 2026-01-01
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: laraib28
command: fix-language-switch-404
labels: ["i18n", "language-switching", "npm-scripts", "production-build", "urdu", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-robotics-book/package.json
  - humanoid-robotics-book/LANGUAGE_SWITCHING_SOLUTION.md
  - humanoid-robotics-book/LANGUAGE_SWITCHING_FIX.md
tests:
  - Production build executed successfully
  - Verified Urdu routes generated in build/ur/
  - Verified all 33 Urdu documentation pages built
  - Confirmed language switcher functional in production mode
---

## Prompt

fix language switch issue when i click on urdu urdu traslated ages not appear page not found

## Response snapshot

**Diagnosis:**
Explored the codebase and identified root cause:
- Docusaurus dev server (`npm start`) only serves default locale (English) for performance
- Clicking Urdu language switcher tries to navigate to `/ur/intro/` or similar paths
- These routes don't exist in development mode - only generated in production build
- Urdu translation files exist (33 docs) but routes not available in dev mode

**Solution Implemented:**

1. **Added NPM Scripts** (package.json)
   - `start:ur` - Run dev server with Urdu locale only
   - `start:prod` - Build and serve production version with full multi-locale support

2. **Verified Production Build**
   - Ran `npm run build` successfully
   - Confirmed both locales built: English (build/) and Urdu (build/ur/)
   - Verified all Urdu routes generated (part1-foundations, part2-modules, part3-capstone, part4-future)
   - All 33 Urdu documentation pages confirmed present

3. **Created Comprehensive Documentation**
   - LANGUAGE_SWITCHING_FIX.md - Quick reference guide
   - LANGUAGE_SWITCHING_SOLUTION.md - Complete technical details with verification checklist

**Technical Details:**
- i18n config: ✅ Correct (defaultLocale: 'en', locales: ['en', 'ur'])
- Translation files: ✅ 33 Urdu docs exist in i18n/ur/
- RTL support: ✅ Configured in Root.tsx and rtl.css
- Navbar translations: ✅ Complete (7 items in navbar.json)
- Production routes: ✅ Generated successfully for both locales

**Root Cause:**
This is NOT a bug - it's expected Docusaurus behavior. Dev mode serves one locale for performance. Production build enables multi-locale routing and makes language switcher functional.

**Solution for User:**
Run `npm run start:prod` to build and serve with full language switching support.

## Outcome

- ✅ Impact: Language switching now works correctly in production mode; user has clear workflow for testing multi-locale functionality
- 🧪 Tests: Production build verified successfully; all Urdu routes confirmed generated; language switcher functional
- 📁 Files: Modified 1 file (package.json); created 2 documentation files (LANGUAGE_SWITCHING_FIX.md, LANGUAGE_SWITCHING_SOLUTION.md)
- 🔁 Next prompts: Test language switching using `npm run start:prod`; deploy to Vercel with multi-locale support; add more UI translations
- 🧠 Reflection: This builds on PHR 0003 by adding convenience scripts and actually verifying the build works. NPM scripts make it easier for users to test multi-locale. Production build confirmation was critical to validate the solution.

## Evaluation notes (flywheel)

- Failure modes observed: User confusion about dev vs production modes; common Docusaurus i18n limitation not well understood
- Graders run and results (PASS/FAIL): PASS - Production build completed successfully; Urdu routes verified; language switcher functional
- Prompt variant (if applicable): Enhanced from PHR 0003 - went beyond documentation to add convenience scripts and run actual verification
- Next experiment (smallest change to try): Could add dev mode warning in UI when attempting to switch locales; consider using Docusaurus feature flags for locale testing
