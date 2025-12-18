# Implementation Plan: Urdu Translation Support

**Branch**: `002-urdu-i18n-support` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-urdu-i18n-support/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Enable complete Urdu translation support for the Humanoid Robotics documentation site, allowing users to view all content in Urdu with RTL layout, switch languages via a navbar toggle, and interact with the chatbot in Urdu. The implementation uses Docusaurus i18n for content localization, integrates language detection with the existing chatbot via Gemini API, and supports URL-based language routing (/ for English, /ur/ for Urdu).

## Technical Context

**Language/Version**: TypeScript 5.6.2, Node.js 20.x LTS
**Primary Dependencies**: Docusaurus 3.9.2, React 19.0.0, @docusaurus/preset-classic 3.9.2, Gemini API (for chatbot translation)
**Storage**: Browser localStorage for language preference persistence
**Testing**: TypeScript type checking, Docusaurus build validation, manual testing for RTL layout and language switching
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge - last 2 versions), deployed to Vercel
**Project Type**: Web application (Docusaurus static site with React components)
**Performance Goals**: < 2 seconds language switching, < 1 second chatbot response detection
**Constraints**: RTL layout must not break existing LTR layout, all translations must be ready before deploying toggle
**Scale/Scope**: ~24 documentation pages across 4 parts, all UI components and labels, chatbot integration

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Content Accuracy and Verification ✅ PASS
- Translation content will be provided by qualified translators (assumption documented in spec)
- All code examples and technical terms will remain in English per industry standards
- RTL layout implementation will be tested across target browsers
- Chatbot language detection will be validated with test cases

### Clarity for Target Audience ✅ PASS
- Documentation will maintain same clarity in both English and Urdu
- UI elements will have parallel translations
- Language toggle will be intuitive and visible

### Reproducibility (NON-NEGOTIABLE) ✅ PASS
- Docusaurus i18n is well-documented with reproducible setup steps
- Translation workflow will be documented in quickstart.md
- All configuration changes will be version-controlled
- RTL CSS will be testable by switching language toggle

### Spec-Driven Development Integration ✅ PASS
- This plan follows spec → plan → tasks workflow
- All decisions will be traceable through artifacts
- Constitution principles guide implementation choices

### Version Control and Collaboration ✅ PASS
- All changes in feature branch: 002-urdu-i18n-support
- Atomic commits for each component (config, translations, UI, chatbot)
- Docusaurus build must pass before merging

### Deployment and Accessibility ✅ PASS
- Docusaurus build validation required
- RTL layout supports accessibility (WCAG 2.1 Level AA maintained)
- Language switching via URL supports bookmarking and sharing
- Vercel deployment pipeline will handle /ur/ routing automatically

**Gate Status**: ✅ ALL GATES PASSED - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/002-urdu-i18n-support/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── chatbot-api.yaml # Chatbot API contract with language parameter
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
humanoid-robotics-book/
├── docusaurus.config.ts         # Update i18n config to add 'ur' locale
├── i18n/                        # NEW: Docusaurus i18n directory
│   └── ur/                      # NEW: Urdu translations
│       ├── docusaurus-plugin-content-docs/  # NEW: Docs translations
│       │   └── current/         # NEW: Current version docs in Urdu
│       │       ├── part1-foundations/
│       │       ├── part2-modules/
│       │       ├── part3-capstone/
│       │       └── part4-future/
│       ├── docusaurus-theme-classic/  # NEW: UI translations
│       │   ├── navbar.json      # Navbar labels in Urdu
│       │   └── footer.json      # Footer labels in Urdu
│       └── code.json            # Common UI strings in Urdu
├── src/
│   ├── components/
│   │   ├── HomepageFeatures/
│   │   └── LanguageToggle/      # NEW: Language switcher component
│   │       ├── index.tsx        # Language toggle logic
│   │       └── styles.module.css # Toggle styling (RTL-aware)
│   ├── services/
│   │   ├── authService.ts       # Existing
│   │   └── chatService.ts       # NEW: Chatbot API with language parameter
│   ├── hooks/
│   │   └── useLanguage.ts       # NEW: Language preference hook
│   └── css/
│       ├── custom.css           # Existing
│       └── rtl.css              # NEW: RTL-specific styles
├── static/
│   └── img/                     # Existing images (no changes)
└── docs/                        # Existing English docs (no changes)
    ├── part1-foundations/
    ├── part2-modules/
    ├── part3-capstone/
    └── part4-future/
```

**Structure Decision**: Web application structure (Option 2) with frontend-only changes. The chatbot backend API is external (assumed to exist per spec), so we only need to update the frontend service layer to pass the language parameter. The i18n/ directory follows Docusaurus conventions for translation organization.

## Complexity Tracking

> **No violations detected. This section is intentionally left empty.**

All constitution gates passed without justification needed. The implementation follows standard Docusaurus i18n patterns, maintains existing architecture, and introduces minimal complexity (language toggle component, RTL CSS, translation files).

