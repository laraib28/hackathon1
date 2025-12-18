# Quickstart Guide: Urdu Translation Support

**Feature**: 002-urdu-i18n-support | **Date**: 2025-12-09 | **Phase**: 1 (Implementation Guide)

## Purpose

This guide provides step-by-step instructions for developers to implement Urdu translation support in the Humanoid Robotics documentation site. Follow these steps in order to add i18n configuration, translations, RTL layout, and chatbot language support.

---

## Prerequisites

- Node.js 20.x or higher installed
- Familiarity with Docusaurus, React, and TypeScript
- Git repository cloned and dependencies installed (`npm install`)
- Urdu translation content ready (or use placeholders for testing)

---

## Implementation Steps

### Step 1: Configure Docusaurus i18n

**File**: `humanoid-robotics-book/docusaurus.config.ts`

**Action**: Update the i18n configuration to add Urdu locale

```typescript
// Before:
i18n: {
  defaultLocale: 'en',
  locales: ['en'],
},

// After:
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
      label: 'اردو',  // "Urdu" in Urdu script
      direction: 'rtl',
      htmlLang: 'ur-PK',
    },
  },
},
```

**Test**:
```bash
npm run build
# Should build successfully with no errors
```

---

### Step 2: Generate Translation Files

**Action**: Use Docusaurus CLI to generate empty translation files

```bash
cd humanoid-robotics-book
npm run write-translations -- --locale ur
```

**Expected Output**:
```text
✔ Created i18n/ur/code.json
✔ Created i18n/ur/docusaurus-theme-classic/navbar.json
✔ Created i18n/ur/docusaurus-theme-classic/footer.json
✔ Created i18n/ur/docusaurus-plugin-content-docs/current.json
```

**File Structure**:
```text
humanoid-robotics-book/
└── i18n/
    └── ur/
        ├── code.json
        ├── docusaurus-theme-classic/
        │   ├── navbar.json
        │   ├── footer.json
        │   └── docs.json
        └── docusaurus-plugin-content-docs/
            └── current.json
```

---

### Step 3: Translate UI Strings

**File**: `i18n/ur/code.json`

**Action**: Translate common UI strings

```json
{
  "theme.ErrorPageContent.title": "یہ صفحہ کریش ہو گیا۔",
  "theme.NotFound.title": "صفحہ نہیں ملا",
  "theme.NotFound.p1": "ہم وہ نہیں ڈھونڈ سکے جو آپ تلاش کر رہے تھے۔",
  "theme.docs.paginator.previous": "پچھلا",
  "theme.docs.paginator.next": "اگلا",
  "theme.common.editThisPage": "یہ صفحہ ترمیم کریں",
  "theme.common.skipToMainContent": "مرکزی مواد پر جائیں",
  "theme.docs.sidebar.collapseButtonTitle": "سائیڈبار بند کریں",
  "theme.docs.sidebar.expandButtonTitle": "سائیڈبار کھولیں",
  "theme.SearchBar.label": "تلاش کریں"
}
```

**File**: `i18n/ur/docusaurus-theme-classic/navbar.json`

```json
{
  "title": "ہیومنائیڈ روبوٹکس اور فزیکل اے آئی",
  "logo.alt": "ہیومنائیڈ روبوٹکس لوگو"
}
```

**Test**:
```bash
npm run start -- --locale ur
# Visit http://localhost:3000/ur/ and verify UI strings are in Urdu
```

---

### Step 4: Translate Documentation Content

**Action**: Copy English docs to Urdu directory and translate

```bash
# Create Urdu docs directory structure
mkdir -p i18n/ur/docusaurus-plugin-content-docs/current

# Copy English docs as starting point
cp -r docs/* i18n/ur/docusaurus-plugin-content-docs/current/
```

**Example Translation** (`i18n/ur/docusaurus-plugin-content-docs/current/part1-foundations/intro.md`):

```markdown
---
id: intro
title: بنیادی تصورات
sidebar_position: 1
---

# فزیکل اے آئی اور ہیومنائیڈ روبوٹکس میں خوش آمدید

یہ کتاب آپ کو جسمانی ذہانت (Physical AI) اور ہیومنائیڈ روبوٹکس کی دنیا میں لے جائے گی۔

## اس کتاب میں آپ کیا سیکھیں گے

- ROS2 کے ساتھ روبوٹ سافٹ ویئر کی بنیادی باتیں
- ڈیجیٹل ٹوئن اور سمولیشن
- NVIDIA Isaac کا استعمال کرتے ہوئے اعلیٰ تصورات
- Vision-Language-Action (VLA) ماڈلز

<!-- Continue translating content... -->
```

**Important Notes**:
- Keep code blocks in English (standard practice)
- Keep technical terms like "ROS2", "API", "GitHub" in English
- Translate explanatory text and headings
- Update internal links to use `/ur/` prefix where appropriate

---

### Step 5: Add RTL CSS Support

**File**: `humanoid-robotics-book/src/css/rtl.css` (create new file)

**Action**: Add RTL-specific styles

```css
/* RTL Layout Adjustments */

/* Keep code blocks LTR even in RTL pages */
[dir="rtl"] pre,
[dir="rtl"] code {
  direction: ltr;
  text-align: left;
}

/* Keep URLs LTR */
[dir="rtl"] a[href^="http"] {
  direction: ltr;
  display: inline-block;
}

/* Adjust sidebar for RTL */
[dir="rtl"] .theme-doc-sidebar-container {
  border-left: none;
  border-right: 1px solid var(--ifm-toc-border-color);
}

/* Fix navbar alignment */
[dir="rtl"] .navbar__items--right {
  flex-direction: row-reverse;
}

/* Adjust pagination buttons */
[dir="rtl"] .pagination-nav__link--next .pagination-nav__label::after {
  content: '←';
  margin-right: 0.5rem;
  margin-left: 0;
}

[dir="rtl"] .pagination-nav__link--prev .pagination-nav__label::before {
  content: '→';
  margin-left: 0.5rem;
  margin-right: 0;
}
```

**File**: `humanoid-robotics-book/docusaurus.config.ts`

**Action**: Import RTL CSS

```typescript
presets: [
  [
    'classic',
    {
      docs: {
        sidebarPath: './sidebars.ts',
        routeBasePath: '/',
      },
      blog: false,
      theme: {
        customCss: [
          './src/css/custom.css',
          './src/css/rtl.css',  // Add this line
        ],
      },
    } satisfies Preset.Options,
  ],
],
```

**Test**:
```bash
npm run start -- --locale ur
# Check RTL layout on http://localhost:3000/ur/
```

---

### Step 6: Create Language Toggle Component

**File**: `humanoid-robotics-book/src/components/LanguageToggle/index.tsx` (create new file)

```tsx
import React, { useState } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

const LANGUAGES = {
  en: { label: 'English', flag: '🇬🇧' },
  ur: { label: 'اردو', flag: '🇵🇰' },
};

export default function LanguageToggle(): JSX.Element {
  const history = useHistory();
  const location = useLocation();
  const [isOpen, setIsOpen] = useState(false);

  // Detect current locale from URL
  const currentLocale = location.pathname.startsWith('/ur/') ? 'ur' : 'en';

  const switchLanguage = (newLocale: 'en' | 'ur') => {
    // Save preference to localStorage
    const preference = {
      locale: newLocale,
      timestamp: Date.now(),
      version: '1.0.0',
    };
    localStorage.setItem('docusaurus.locale', JSON.stringify(preference));

    // Build new path
    let newPath = location.pathname;
    if (newLocale === 'ur' && !newPath.startsWith('/ur/')) {
      newPath = `/ur${newPath === '/' ? '' : newPath}`;
    } else if (newLocale === 'en' && newPath.startsWith('/ur/')) {
      newPath = newPath.replace(/^\/ur/, '') || '/';
    }

    // Navigate to new locale
    history.push(newPath + location.search + location.hash);
    setIsOpen(false);
  };

  return (
    <div className={styles.languageToggle}>
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Switch language"
        aria-expanded={isOpen}
      >
        <span>{LANGUAGES[currentLocale].flag}</span>
        <span>{LANGUAGES[currentLocale].label}</span>
        <span className={styles.arrow}>▼</span>
      </button>

      {isOpen && (
        <div className={styles.dropdown}>
          {Object.entries(LANGUAGES).map(([locale, { label, flag }]) => (
            <button
              key={locale}
              className={`${styles.dropdownItem} ${
                locale === currentLocale ? styles.active : ''
              }`}
              onClick={() => switchLanguage(locale as 'en' | 'ur')}
            >
              <span>{flag}</span>
              <span>{label}</span>
            </button>
          ))}
        </div>
      )}
    </div>
  );
}
```

**File**: `humanoid-robotics-book/src/components/LanguageToggle/styles.module.css` (create new file)

```css
.languageToggle {
  position: relative;
  display: inline-block;
}

.toggleButton {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.5rem 1rem;
  background: var(--ifm-color-emphasis-100);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 4px;
  cursor: pointer;
  font-size: 0.9rem;
  transition: all 0.2s;
}

.toggleButton:hover {
  background: var(--ifm-color-emphasis-200);
}

.arrow {
  font-size: 0.7rem;
  transition: transform 0.2s;
}

.dropdown {
  position: absolute;
  top: 100%;
  right: 0;
  margin-top: 0.5rem;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 4px;
  box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
  min-width: 150px;
  z-index: 1000;
}

[dir="rtl"] .dropdown {
  left: 0;
  right: auto;
}

.dropdownItem {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  width: 100%;
  padding: 0.75rem 1rem;
  border: none;
  background: none;
  cursor: pointer;
  text-align: left;
  transition: background 0.2s;
}

[dir="rtl"] .dropdownItem {
  text-align: right;
}

.dropdownItem:hover {
  background: var(--ifm-color-emphasis-100);
}

.dropdownItem.active {
  background: var(--ifm-color-emphasis-200);
  font-weight: bold;
}
```

**File**: `humanoid-robotics-book/docusaurus.config.ts`

**Action**: Add language toggle to navbar

```typescript
themeConfig: {
  navbar: {
    title: 'Humanoid Robotics & Physical AI',
    logo: {
      alt: 'Humanoid Robotics Logo',
      src: 'img/logo.svg',
    },
    items: [
      // ... existing navbar items ...
      {
        type: 'custom-languageToggle',  // Custom navbar item type
        position: 'right',
      },
    ],
  },
  // ... other theme config ...
} satisfies Preset.ThemeConfig,
```

**Note**: You may need to swizzle the navbar component to add the custom language toggle. Alternatively, use Docusaurus built-in `type: 'localeDropdown'`:

```typescript
items: [
  {
    type: 'localeDropdown',
    position: 'right',
  },
],
```

**Test**:
```bash
npm run start
# Click language toggle, verify it switches between EN and UR
# Verify preference persists on page refresh
```

---

### Step 7: Add Chatbot Language Support

**File**: `humanoid-robotics-book/src/services/chatService.ts` (create or update)

```typescript
interface ChatRequest {
  query: string;
  language: 'en' | 'ur';
  sessionId?: string;
  context?: string;
}

interface ChatResponse {
  answer: string;
  language: 'en' | 'ur';
  confidence?: number;
  sources?: string[];
  error?: string;
}

/**
 * Detect language from user query using Unicode pattern matching
 */
function detectLanguage(text: string): 'en' | 'ur' {
  // Urdu Unicode range: U+0600 to U+06FF (Arabic/Urdu script)
  const urduPattern = /[\u0600-\u06FF]/;
  return urduPattern.test(text) ? 'ur' : 'en';
}

/**
 * Send chat message to backend API
 */
export async function sendChatMessage(
  query: string,
  currentLocale: 'en' | 'ur',
  sessionId?: string
): Promise<ChatResponse> {
  // Detect language from query content (prioritize over toggle)
  const detectedLanguage = detectLanguage(query);
  const targetLanguage = detectedLanguage || currentLocale;

  const requestBody: ChatRequest = {
    query,
    language: targetLanguage,
    sessionId,
    context: window.location.href,
  };

  try {
    const response = await fetch('https://api.example.com/v1/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        // Add API key if required
        // 'X-API-Key': process.env.CHAT_API_KEY,
      },
      body: JSON.stringify(requestBody),
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${response.statusText}`);
    }

    const data: ChatResponse = await response.json();
    return data;
  } catch (error) {
    console.error('Chat API error:', error);
    return {
      answer: targetLanguage === 'ur'
        ? 'معذرت، چیٹ بوٹ فی الحال دستیاب نہیں ہے۔'
        : 'Sorry, the chatbot is currently unavailable.',
      language: targetLanguage,
      error: error.message,
    };
  }
}
```

**File**: `humanoid-robotics-book/src/hooks/useLanguage.ts` (create new file)

```typescript
import { useLocation } from '@docusaurus/router';
import { useEffect, useState } from 'react';

/**
 * Custom hook to get current language and language preference
 */
export function useLanguage(): 'en' | 'ur' {
  const location = useLocation();
  const [locale, setLocale] = useState<'en' | 'ur'>('en');

  useEffect(() => {
    // Detect locale from URL
    const urlLocale = location.pathname.startsWith('/ur/') ? 'ur' : 'en';
    setLocale(urlLocale);

    // Update localStorage
    const preference = {
      locale: urlLocale,
      timestamp: Date.now(),
      version: '1.0.0',
    };
    localStorage.setItem('docusaurus.locale', JSON.stringify(preference));
  }, [location.pathname]);

  return locale;
}
```

**Usage in Chat Component**:
```tsx
import { useLanguage } from '@site/src/hooks/useLanguage';
import { sendChatMessage } from '@site/src/services/chatService';

function ChatWidget() {
  const currentLocale = useLanguage();
  const [messages, setMessages] = useState([]);

  const handleSendMessage = async (userQuery: string) => {
    const response = await sendChatMessage(userQuery, currentLocale);
    setMessages([
      ...messages,
      { role: 'user', content: userQuery, language: currentLocale },
      { role: 'assistant', content: response.answer, language: response.language },
    ]);
  };

  return (
    <div>
      {/* Chat UI implementation */}
    </div>
  );
}
```

---

### Step 8: Build and Deploy

**Build for Production**:
```bash
npm run build
# This generates static files for both EN and UR locales
```

**Expected Output**:
```text
build/
├── index.html                 # English homepage
├── part1-foundations/
│   └── intro/
│       └── index.html         # English page
├── ur/
│   ├── index.html             # Urdu homepage
│   └── part1-foundations/
│       └── intro/
│           └── index.html     # Urdu page
└── ...
```

**Deploy to Vercel** (automatic):
```bash
git add .
git commit -m "feat: add Urdu translation support with language toggle and chatbot"
git push origin 002-urdu-i18n-support
# Vercel will automatically build and deploy
```

**Verify Deployment**:
- Visit `https://hackathon1-9y2e.vercel.app/` (English)
- Visit `https://hackathon1-9y2e.vercel.app/ur/` (Urdu)
- Test language toggle switches between EN and UR
- Test chatbot responds in correct language

---

## Testing Checklist

### Functional Testing

- [ ] English homepage loads correctly at `/`
- [ ] Urdu homepage loads correctly at `/ur/`
- [ ] Language toggle is visible in navbar
- [ ] Clicking language toggle switches between EN and UR
- [ ] Language preference persists after page refresh
- [ ] All UI strings display in selected language
- [ ] Documentation pages display in selected language
- [ ] Code blocks remain LTR in Urdu pages
- [ ] Chatbot detects Urdu queries correctly
- [ ] Chatbot responds in Urdu when requested
- [ ] Chatbot responds in English when requested
- [ ] Internal links work in both languages
- [ ] External links work in both languages

### RTL Layout Testing

- [ ] Text flows right-to-left on Urdu pages
- [ ] Sidebar appears on left side (mirrored) on Urdu pages
- [ ] Navigation buttons are in correct RTL positions
- [ ] Images and diagrams display correctly
- [ ] Code blocks remain LTR
- [ ] Search box works in RTL layout
- [ ] Mobile responsive layout works for RTL

### Browser Testing

- [ ] Chrome (latest)
- [ ] Firefox (latest)
- [ ] Safari (latest)
- [ ] Edge (latest)
- [ ] Mobile Safari (iOS)
- [ ] Mobile Chrome (Android)

### Performance Testing

- [ ] Language switching < 2 seconds
- [ ] Page load time acceptable for both locales
- [ ] No console errors or warnings
- [ ] Lighthouse score ≥ 90 for Performance

---

## Troubleshooting

### Issue: Urdu text displays as boxes (□□□)

**Cause**: Missing Urdu font support

**Solution**: Add Urdu-compatible font in `src/css/custom.css`:
```css
@import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;700&display=swap');

[lang="ur"] {
  font-family: 'Noto Nastaliq Urdu', serif;
}
```

### Issue: Language toggle doesn't persist after refresh

**Cause**: localStorage not being read correctly

**Solution**: Check browser console for errors. Ensure `useLanguage` hook is running on page load.

### Issue: RTL layout is broken on some components

**Cause**: Component uses hardcoded left/right CSS properties

**Solution**: Update component to use CSS logical properties (`margin-inline-start` instead of `margin-left`).

### Issue: Chatbot language detection is inaccurate

**Cause**: Mixed English/Urdu text in query

**Solution**: Improve detection heuristic to check percentage of Urdu characters:
```typescript
function detectLanguage(text: string): 'en' | 'ur' {
  const urduChars = (text.match(/[\u0600-\u06FF]/g) || []).length;
  const totalChars = text.replace(/\s/g, '').length;
  return urduChars / totalChars > 0.3 ? 'ur' : 'en';
}
```

---

## Next Steps

After completing this quickstart:

1. **Review** the generated translation files and ensure quality
2. **Test** thoroughly across browsers and devices
3. **Document** any custom translations or deviations from this guide
4. **Monitor** user feedback and chatbot language detection accuracy
5. **Iterate** on translations based on user feedback

---

**Quickstart Guide Completed**: 2025-12-09
**Reviewed By**: Claude (AI Assistant)
**Status**: ✅ Ready for Implementation (Phase 2: Tasks)
