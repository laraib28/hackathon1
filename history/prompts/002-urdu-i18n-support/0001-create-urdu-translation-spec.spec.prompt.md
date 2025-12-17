---
id: 0001
title: create-urdu-translation-spec
stage: spec
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-urdu-i18n-support
branch: 002-urdu-i18n-support
user: user
command: /sp.specify
labels: ["i18n", "urdu", "translation", "specification", "rtl", "chatbot"]
links:
  spec: specs/002-urdu-i18n-support/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-urdu-i18n-support/spec.md
  - specs/002-urdu-i18n-support/checklists/requirements.md
tests:
  - Specification quality checklist validation (all items passed)
---

## Prompt

# Feature: Urdu Translation Support

## Goals
- Entire documentation must support Urdu.
- User should be able to toggle EN/UR from navbar.
- Chatbot should detect question language and answer in same language.
- All UI labels (buttons, headings) Urdu version included.

## Requirements
1. Add Urdu translations to Docusaurus using i18n.
2. Create folder: i18n/ur/docusaurus-plugin-content-docs/current
3. Backend:
   - Add parameter `target_language="ur"` in chat endpoint
   - If selected text comes in English → translate to Urdu before responding
4. Frontend:
   - Add language toggle in Navbar
   - Default remains English
   - Urdu version is RTL

## Acceptance Criteria
- Selecting Urdu shows fully translated pages.
- Chatbot responds in Urdu when toggle is ON or question is in Urdu.

## Response snapshot

Created comprehensive feature specification for Urdu translation support with:

**Branch & Files**:
- Created branch: `002-urdu-i18n-support`
- Created specification: `specs/002-urdu-i18n-support/spec.md`
- Created quality checklist: `specs/002-urdu-i18n-support/checklists/requirements.md`

**Specification Structure**:
- 4 prioritized user stories (2 P1, 2 P2) with independent test criteria
- 16 functional requirements (FR-001 to FR-016)
- 8 measurable, technology-agnostic success criteria
- 7 edge cases identified
- Complete sections: Assumptions, Dependencies, Constraints, Out of Scope
- Key entities defined (Translation Bundle, User Language Preference, etc.)

**Quality Validation**:
- All checklist items passed (16/16)
- No [NEEDS CLARIFICATION] markers (resolved fallback behavior)
- No implementation details (made technology-agnostic)
- Testable and unambiguous requirements
- Ready for planning phase

**Key Decisions**:
- English content shown as fallback when Urdu translation missing
- Chatbot detects language from input OR respects toggle setting
- RTL layout for Urdu, LTR for English
- Language preference persists across sessions
- URL-based language selection supported

## Outcome

- ✅ Impact: Feature specification complete and validated. Ready for `/sp.plan` or `/sp.clarify`
- 🧪 Tests: Specification quality checklist - all 16 validation criteria passed
- 📁 Files: 2 files created (spec.md, checklists/requirements.md)
- 🔁 Next prompts: `/sp.clarify` (optional for targeted questions) or `/sp.plan` (for implementation planning)
- 🧠 Reflection: Specification successfully balances detail with technology-agnosticism. User stories prioritized for MVP approach. All requirements testable.

## Evaluation notes (flywheel)

- Failure modes observed: Initial spec included implementation details (Docusaurus-specific). Fixed by making requirements technology-agnostic.
- Graders run and results (PASS/FAIL): Specification quality checklist - PASS (all items validated)
- Prompt variant (if applicable): Standard /sp.specify workflow with user-provided detailed requirements
- Next experiment (smallest change to try): Consider if user stories could be broken down further for even smaller MVPs
