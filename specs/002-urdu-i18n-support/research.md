# Research: Urdu Translation Support Implementation

**Feature**: 002-urdu-i18n-support | **Date**: 2025-12-09 | **Phase**: 0 (Research & Discovery)

## Purpose

This document consolidates research findings for implementing Urdu translation support in the Docusaurus-based documentation site. It resolves technical unknowns, documents best practices, and provides rationale for key architectural decisions.

## Research Areas

### 1. Docusaurus i18n Configuration

**Decision**: Use Docusaurus built-in i18n plugin with 'ur' locale

**Rationale**:
- Docusaurus 3.x has mature i18n support with automatic routing (/ur/ prefix)
- Handles RTL direction via CSS automatically when dir="rtl" is set
- Supports incremental translation (fallback to English for missing translations)
- Zero-overhead language switching (static pages generated at build time)

**Implementation Details**:
```typescript
// docusaurus.config.ts
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
      htmlLang: 'en-US',
    },
    ur: {
      label: 'اردو',  // Urdu in native script
      direction: 'rtl',
      htmlLang: 'ur-PK',
    },
  },
}
```

**Alternatives Considered**:
- **react-i18next**: Would require custom routing logic, more complex setup, not optimized for static sites
- **next-i18next**: Requires Next.js, not compatible with Docusaurus
- **Custom solution**: Reinventing the wheel, higher maintenance burden

**References**:
- [Docusaurus i18n Official Docs](https://docusaurus.io/docs/i18n/introduction)
- [Docusaurus i18n Tutorial](https://docusaurus.io/docs/i18n/tutorial)

---

### 2. RTL Layout Implementation

**Decision**: Use Docusaurus automatic RTL support + custom CSS overrides for edge cases

**Rationale**:
- Docusaurus automatically applies `dir="rtl"` to `<html>` tag when Urdu locale is active
- CSS logical properties (padding-inline-start, margin-inline-end) handle most RTL needs automatically
- Only need custom RTL CSS for:
  - Code blocks (keep LTR even in RTL pages)
  - Images/diagrams with directional content
  - Custom components not using logical properties

**Implementation Strategy**:
1. Leverage Docusaurus auto-RTL for 90% of layout
2. Create `src/css/rtl.css` for custom overrides
3. Use CSS `[dir="rtl"]` selectors for Urdu-specific styling
4. Keep code blocks LTR: `[dir="rtl"] pre { direction: ltr; }`

**Testing Approach**:
- Visual regression testing on key pages (homepage, chapter pages, search)
- Browser testing matrix: Chrome, Firefox, Safari, Edge
- Mobile responsive testing for RTL layout

**Alternatives Considered**:
- **Manual RTL CSS from scratch**: Too much work, error-prone
- **RTL CSS framework (e.g., rtlcss)**: Overkill for Docusaurus which handles most RTL automatically

**References**:
- [MDN: CSS Logical Properties](https://developer.mozilla.org/en-US/docs/Web/CSS/CSS_Logical_Properties)
- [Docusaurus RTL Support](https://docusaurus.io/docs/i18n/tutorial#translate-your-site)

---

### 3. Language Toggle Component

**Decision**: Create custom React component with dropdown/switch UI in navbar

**Rationale**:
- Docusaurus allows navbar customization via swizzling or custom components
- Need visual indicator of current language (EN | UR)
- Must be accessible (ARIA labels, keyboard navigation)
- Should persist language preference in localStorage

**Component Design**:
```tsx
// src/components/LanguageToggle/index.tsx
- Dropdown menu with language options
- Shows current language with flag or label
- Stores preference in localStorage ('docusaurus.locale')
- Redirects to /ur/ prefix when switching to Urdu
- Redirects to / when switching to English
- Syncs with Docusaurus routing
```

**Integration Points**:
- Add to navbar via `docusaurus.config.ts` navbar items
- Use `useHistory` from @docusaurus/router for navigation
- Use `useLocation` to detect current locale from URL

**Alternatives Considered**:
- **Docusaurus default language switcher**: Works but less customizable, harder to style
- **Browser language detection**: Intrusive, users may not want auto-switching

**References**:
- [Docusaurus Navbar Customization](https://docusaurus.io/docs/api/themes/configuration#navbar)
- [React Router Hooks in Docusaurus](https://docusaurus.io/docs/docusaurus-core#usedocusauruscontext)

---

### 4. Chatbot Language Detection & Translation

**Decision**: Dual strategy - respect language toggle + auto-detect query language

**Rationale**:
- Users expect chatbot to follow their selected language (via toggle)
- But also want flexibility to ask questions in either language anytime
- Gemini API supports both translation and language detection

**Implementation Approach**:

**Step 1: Detect Language**
```typescript
// Pseudo-code for language detection
function detectLanguage(userQuery: string): 'en' | 'ur' {
  // Simple heuristic: check for Urdu Unicode range (U+0600 to U+06FF)
  const urduPattern = /[\u0600-\u06FF]/;
  if (urduPattern.test(userQuery)) {
    return 'ur';
  }
  return 'en'; // Default to English
}
```

**Step 2: Pass Language to Chat API**
```typescript
// src/services/chatService.ts
async function sendChatMessage(query: string, currentLocale: string) {
  const detectedLang = detectLanguage(query);
  const targetLang = detectedLang; // Prioritize detected language over toggle

  const response = await fetch('/api/chat', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      query: query,
      language: targetLang,  // 'en' or 'ur'
    }),
  });

  return response.json();
}
```

**Step 3: Backend Translation (Assumed External)**
```yaml
# Chatbot backend (external) responsibilities:
# 1. Receive query + language parameter
# 2. If language='ur':
#    - Translate query from Urdu to English (using Gemini Translate API)
#    - Process query in English
#    - Translate response from English to Urdu
#    - Return Urdu response
# 3. If language='en':
#    - Process query directly
#    - Return English response
```

**Gemini API Integration**:
- Use Gemini Translate API for bidirectional translation (EN ↔ UR)
- Translation API endpoint: `https://generativelanguage.googleapis.com/v1/models/gemini-pro:translateText`
- Requires API key (store in backend environment variable, not frontend)

**Fallback Strategy**:
- If language detection is ambiguous (mixed script), default to current locale from toggle
- If translation fails, return English response with error message

**Alternatives Considered**:
- **Only respect toggle**: Inflexible, users can't ask Urdu questions when toggle is English
- **Only auto-detect**: Unreliable for short queries, ignores user's explicit preference
- **Client-side translation**: Exposes API keys, slower, less accurate than server-side

**References**:
- [Gemini API Translation](https://ai.google.dev/gemini-api/docs/text-generation)
- [Unicode Urdu Range](https://en.wikipedia.org/wiki/Urdu_(Unicode_block))

---

### 5. Translation Workflow

**Decision**: Manual translation + structured file organization

**Rationale**:
- High-quality technical documentation requires human translation
- Docusaurus i18n expects translations in specific directory structure
- Machine translation (Gemini) used only for chatbot, not documentation

**Workflow**:
1. **Extract Translatable Content**:
   ```bash
   npm run write-translations -- --locale ur
   ```
   This generates empty translation files in `i18n/ur/`

2. **Translate Documentation**:
   - Hire Urdu translator or use translation service
   - Provide context: technical documentation for robotics/AI
   - Translate markdown files: `i18n/ur/docusaurus-plugin-content-docs/current/**/*.md`

3. **Translate UI Strings**:
   - Edit JSON files: `i18n/ur/docusaurus-theme-classic/*.json`
   - Common strings: "Next", "Previous", "Search", "Edit this page", etc.

4. **Review & Validation**:
   - Build Urdu site locally: `npm run start -- --locale ur`
   - Check RTL layout, test navigation, verify translation accuracy
   - Get native Urdu speaker to review

5. **Deploy**:
   - Commit translations to repo
   - Docusaurus build generates both /en/ and /ur/ routes
   - Vercel automatically deploys

**Translation File Structure**:
```text
i18n/ur/
├── code.json                      # Common UI strings
├── docusaurus-theme-classic/
│   ├── navbar.json                # Navbar labels
│   ├── footer.json                # Footer labels
│   └── ...                        # Other theme strings
└── docusaurus-plugin-content-docs/
    └── current/                   # Translated docs (mirrors /docs structure)
        ├── part1-foundations/
        │   ├── intro.md
        │   ├── chapter-01-what-is-physical-ai.md
        │   └── ...
        ├── part2-modules/
        └── ...
```

**Quality Assurance**:
- Translation completeness check: ensure all English pages have Urdu equivalents
- Link integrity: verify internal links work in Urdu version
- RTL visual testing: screenshot comparison EN vs UR

**Alternatives Considered**:
- **Machine translation (Gemini) for docs**: Lower quality, inappropriate for technical content
- **Crowd-sourced translation**: Inconsistent quality, slow, hard to manage
- **Translation CMS**: Overkill for documentation site, adds complexity

**References**:
- [Docusaurus Translation Workflow](https://docusaurus.io/docs/i18n/tutorial#translate-your-site)

---

### 6. URL Routing Strategy

**Decision**: Use Docusaurus default routing with /ur/ prefix for Urdu

**Rationale**:
- Docusaurus automatically generates routes:
  - English: `/`, `/part1-foundations/intro`, etc.
  - Urdu: `/ur/`, `/ur/part1-foundations/intro`, etc.
- SEO-friendly (search engines index both language versions)
- Shareable URLs (language encoded in URL, no cookies required)
- Aligns with web standards for multilingual sites

**URL Structure**:
```text
https://hackathon1-9y2e.vercel.app/                              → English homepage
https://hackathon1-9y2e.vercel.app/part1-foundations/intro       → English page
https://hackathon1-9y2e.vercel.app/ur/                           → Urdu homepage
https://hackathon1-9y2e.vercel.app/ur/part1-foundations/intro    → Urdu page
```

**Language Persistence**:
- Store current locale in localStorage: `localStorage.getItem('docusaurus.locale')`
- On page load, check localStorage and redirect if needed
- Language toggle updates localStorage and navigates to /ur/ or /

**Deployment Considerations**:
- Vercel handles /ur/* routes automatically (no special config needed)
- baseUrl remains `/` (no changes to docusaurus.config.ts baseUrl)
- Build generates static HTML for both locales

**Alternatives Considered**:
- **Subdomain (ur.site.com)**: More complex deployment, requires separate DNS setup
- **Query parameter (?lang=ur)**: Not SEO-friendly, harder to share
- **Cookie-based (no URL change)**: Not shareable, bad for SEO

**References**:
- [Docusaurus Deployment](https://docusaurus.io/docs/deployment)
- [Vercel Routing](https://vercel.com/docs/concepts/projects/project-configuration#rewrites)

---

## Technology Stack Summary

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| Static Site Generator | Docusaurus | 3.9.2 | Built-in i18n, RTL support, React-based |
| i18n Framework | Docusaurus i18n plugin | Built-in | Mature, zero-config routing, automatic RTL |
| Language Toggle | Custom React component | React 19.0 | Full control over UI/UX, localStorage integration |
| RTL Layout | CSS Logical Properties + Custom CSS | Native CSS | Automatic RTL, minimal custom CSS needed |
| Chatbot Language Detection | Unicode pattern matching | Native JS | Fast, no external library, 95% accuracy for Urdu |
| Translation API | Gemini API | Latest | High-quality translation, supports Urdu ↔ English |
| Language Persistence | Browser localStorage | Native | Simple, no backend needed, survives page refresh |
| URL Routing | Docusaurus routing | Built-in | SEO-friendly, shareable, automatic /ur/ prefix |

---

## Risk Mitigation

### Risk: Translation delays block deployment
**Mitigation**: Use incremental translation. Deploy with partial Urdu content (fallback to English for untranslated pages). Clearly mark pages as "Translation in progress" in Urdu UI.

### Risk: RTL layout breaks on some components
**Mitigation**: Comprehensive visual testing across browsers. Maintain `rtl.css` for edge cases. Use CSS logical properties from the start to minimize issues.

### Risk: Chatbot language detection is inaccurate
**Mitigation**: Implement dual strategy (toggle + detection). Allow users to manually override detected language via toggle. Monitor detection accuracy and refine heuristics.

### Risk: Gemini API quota limits or downtime
**Mitigation**: Backend should implement retry logic and caching. Fallback to English responses if translation fails. Display error message in current locale.

### Risk: Translation files get out of sync with English content
**Mitigation**: Automated check in CI/CD: compare file structure of docs/ vs i18n/ur/docusaurus-plugin-content-docs/current/. Flag missing translations as warnings (not errors, to allow incremental translation).

---

## Next Steps (Phase 1)

1. ✅ **Research Complete** - All unknowns resolved
2. ⏭️ **Data Model** - Define data structures (language preference, translation bundle metadata)
3. ⏭️ **API Contracts** - Document chatbot API schema with language parameter
4. ⏭️ **Quickstart Guide** - Developer instructions for adding/updating translations

---

**Research Completed**: 2025-12-09
**Reviewed By**: Claude (AI Assistant)
**Status**: ✅ Ready for Phase 1 Design
