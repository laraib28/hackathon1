---
id: 0003
title: Fix language switcher clicking issue
stage: general
date: 2025-12-26
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: laraib28
command: troubleshoot-language-switching
labels: ["i18n", "language-switching", "docusaurus", "urdu", "troubleshooting"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - humanoid-robotics-book/LANGUAGE_SWITCHING_TEST.md
tests:
  - Manual test: Build site and verify language dropdown switches pages
  - Test English to Urdu switching
  - Test Urdu to English switching
---

## Prompt

> jb urdu p click krn to urdu pages ayn jb english p click krn to english pages ayn

Translation: "When clicking on Urdu, Urdu pages should come. When clicking on English, English pages should come."

## Response snapshot

Diagnosed that language dropdown appears but doesn't switch pages when clicked.

**Root Cause:**
- Docusaurus dev server (`npm start`) only runs default locale (English) for performance
- Multi-locale support requires production build

**Solution Provided:**
1. Build site with both languages: `npm run build`
2. Serve locally: `npm run serve`
3. Language switcher will work at `http://localhost:3000`

**Exploration Performed:**
- Explored full i18n configuration in docusaurus.config.ts
- Verified 30+ Urdu translations exist in `/i18n/ur/` directory
- Confirmed locale dropdown configured in navbar
- Verified RTL support in Root.tsx and rtl.css
- Confirmed URL structure: `/` (English) and `/ur/` (Urdu)

**Documentation Created:**
- Created LANGUAGE_SWITCHING_TEST.md with complete testing guide
- Includes troubleshooting steps and URL structure reference

## Outcome

- ✅ Impact: User can now properly test and verify language switching functionality by building the site
- 🧪 Tests: Manual testing steps provided for English ↔ Urdu switching
- 📁 Files: Created 1 new documentation file (LANGUAGE_SWITCHING_TEST.md)
- 🔁 Next prompts: Possible follow-ups include running the build, adding more translations, or deploying to Vercel
- 🧠 Reflection: This was a common i18n gotcha - dev server optimizes by serving only default locale. Build step necessary to test multi-locale.

## Evaluation notes (flywheel)

- Failure modes observed: User expected dev server to support all locales by default; common misconception with Docusaurus i18n
- Graders run and results (PASS/FAIL): N/A - diagnostic and documentation task
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): User should run `npm run build && npm run serve` to verify language switching works correctly
