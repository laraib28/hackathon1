---
description: "Task list for Urdu translation support implementation"
---

# Tasks: Urdu Translation Support

**Input**: Design documents from `/specs/002-urdu-i18n-support/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/chatbot-api.yaml

**Tests**: No explicit test tasks included - manual testing will be performed for RTL layout, language switching, and chatbot behavior per quickstart.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `humanoid-robotics-book/` (Docusaurus project root)
- All paths are relative to `humanoid-robotics-book/` directory
- i18n translations: `i18n/ur/`
- React components: `src/components/`
- Services: `src/services/`
- Hooks: `src/hooks/`
- Styles: `src/css/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and Docusaurus i18n configuration

- [x] T001 Update Docusaurus i18n config to add 'ur' locale in humanoid-robotics-book/docusaurus.config.ts
- [x] T002 Generate Urdu translation file structure using `npm run write-translations -- --locale ur`
- [x] T003 [P] Create i18n/ur/ directory structure with subdirectories for docs and theme translations
- [x] T004 [P] Install any additional dependencies needed for RTL support (if applicable)

**Checkpoint**: Docusaurus build succeeds with Urdu locale configured

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core UI translations and foundational components that MUST be complete before user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 [P] Translate common UI strings in i18n/ur/code.json (error messages, navigation, etc.)
- [x] T006 [P] Translate navbar strings in i18n/ur/docusaurus-theme-classic/navbar.json
- [x] T007 [P] Translate footer strings in i18n/ur/docusaurus-theme-classic/footer.json
- [x] T008 [P] Translate docs sidebar strings in i18n/ur/docusaurus-theme-classic/docs.json (if exists)
- [x] T009 Create base RTL CSS file in humanoid-robotics-book/src/css/rtl.css with code block LTR preservation
- [x] T010 Import rtl.css in humanoid-robotics-book/docusaurus.config.ts theme customCss array
- [x] T011 Build Docusaurus with Urdu locale to verify no errors: `npm run build`

**Checkpoint**: Foundation ready - UI strings translated, RTL CSS loaded, build succeeds

---

## Phase 3: User Story 1 - View Documentation in Urdu (Priority: P1) 🎯 MVP

**Goal**: Enable users to view all documentation content in Urdu with proper RTL layout

**Independent Test**: Navigate to `/ur/` route, verify all pages display in Urdu with RTL text direction and fallback to English for missing translations

### Implementation for User Story 1

- [ ] T012 [P] [US1] Copy English docs to i18n/ur/docusaurus-plugin-content-docs/current/ directory
- [ ] T013 [P] [US1] Translate part1-foundations/intro.md in i18n/ur/docusaurus-plugin-content-docs/current/part1-foundations/intro.md
- [ ] T014 [P] [US1] Translate part1-foundations/chapter-01 in i18n/ur/docusaurus-plugin-content-docs/current/part1-foundations/chapter-01-what-is-physical-ai.md
- [ ] T015 [P] [US1] Translate part1-foundations/chapter-02 in i18n/ur/docusaurus-plugin-content-docs/current/part1-foundations/chapter-02-understanding-humanoid-robots.md
- [ ] T016 [P] [US1] Translate part1-foundations/chapter-03 in i18n/ur/docusaurus-plugin-content-docs/current/part1-foundations/chapter-03-why-embodied-intelligence-matters.md
- [ ] T017 [P] [US1] Translate part2-modules/module1-ros2/intro.md in i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module1-ros2/intro.md
- [ ] T018 [P] [US1] Translate part2-modules/module1-ros2/chapter-04 in i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module1-ros2/chapter-04-introduction-to-ros2.md
- [ ] T019 [P] [US1] Translate part2-modules/module1-ros2/chapter-05 in i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module1-ros2/chapter-05-nodes-topics-services-actions.md
- [ ] T020 [P] [US1] Translate part2-modules/module1-ros2/chapter-06 in i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module1-ros2/chapter-06-bridging-python-agents-to-ros.md
- [ ] T021 [P] [US1] Translate part2-modules/module1-ros2/chapter-07 in i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module1-ros2/chapter-07-urdf-modeling-humanoid.md
- [ ] T022 [P] [US1] Translate part2-modules/module2-digital-twin sections in i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module2-digital-twin/
- [ ] T023 [P] [US1] Translate part2-modules/module3-isaac sections in i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module3-isaac/
- [ ] T024 [P] [US1] Translate part2-modules/module4-vla sections in i18n/ur/docusaurus-plugin-content-docs/current/part2-modules/module4-vla/
- [ ] T025 [P] [US1] Translate part3-capstone sections in i18n/ur/docusaurus-plugin-content-docs/current/part3-capstone/
- [ ] T026 [P] [US1] Translate part4-future sections in i18n/ur/docusaurus-plugin-content-docs/current/part4-future/
- [ ] T027 [US1] Build Docusaurus with Urdu content: `npm run build` and verify /ur/ routes generated
- [ ] T028 [US1] Start dev server with Urdu locale: `npm run start -- --locale ur` and manually test navigation
- [ ] T029 [US1] Verify RTL text direction applied correctly on Urdu pages via browser inspection
- [ ] T030 [US1] Test fallback behavior: remove one translation and verify English content shown

**Checkpoint**: At this point, User Story 1 should be fully functional - all docs viewable in Urdu at /ur/ routes with RTL layout

---

## Phase 4: User Story 2 - Switch Between English and Urdu (Priority: P1)

**Goal**: Enable users to switch languages via navbar toggle and persist preference

**Independent Test**: Use language toggle to switch between EN and UR, verify URL changes (/ur/ prefix), preference persists on refresh

### Implementation for User Story 2

- [ ] T031 [P] [US2] Create LanguageToggle component in humanoid-robotics-book/src/components/LanguageToggle/index.tsx
- [ ] T032 [P] [US2] Create LanguageToggle styles in humanoid-robotics-book/src/components/LanguageToggle/styles.module.css
- [ ] T033 [US2] Implement language detection logic from URL pathname in LanguageToggle component
- [ ] T034 [US2] Implement language switching logic with localStorage persistence in LanguageToggle component
- [ ] T035 [US2] Implement dropdown UI with EN and UR options in LanguageToggle component
- [ ] T036 [US2] Add LanguageToggle to navbar in humanoid-robotics-book/docusaurus.config.ts themeConfig.navbar.items
- [ ] T037 [US2] Create useLanguage custom hook in humanoid-robotics-book/src/hooks/useLanguage.ts
- [ ] T038 [US2] Test language toggle switches between / and /ur/ routes correctly
- [ ] T039 [US2] Test language preference persists in localStorage after page refresh
- [ ] T040 [US2] Test language toggle dropdown closes after selection
- [ ] T041 [US2] Test language toggle displays correct current language (EN or UR flag/label)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - users can view Urdu docs and switch languages via navbar

---

## Phase 5: User Story 4 - Experience Proper RTL Layout for Urdu (Priority: P2)

**Goal**: Ensure RTL layout is correctly applied for all Urdu pages with proper mirroring

**Independent Test**: View Urdu pages and verify text flows RTL, UI elements mirrored, code blocks remain LTR

**Note**: US4 is implemented before US3 because RTL CSS is needed for all Urdu content, including chatbot UI

### Implementation for User Story 4

- [ ] T042 [P] [US4] Add RTL-specific CSS for sidebar mirroring in humanoid-robotics-book/src/css/rtl.css
- [ ] T043 [P] [US4] Add RTL-specific CSS for navbar alignment in humanoid-robotics-book/src/css/rtl.css
- [ ] T044 [P] [US4] Add RTL-specific CSS for pagination button mirroring in humanoid-robotics-book/src/css/rtl.css
- [ ] T045 [P] [US4] Add RTL-specific CSS for search box alignment in humanoid-robotics-book/src/css/rtl.css (if applicable)
- [ ] T046 [P] [US4] Add CSS to keep URLs and links LTR within RTL pages in humanoid-robotics-book/src/css/rtl.css
- [ ] T047 [US4] Test RTL layout on Chrome: verify text direction, sidebar position, navbar alignment
- [ ] T048 [US4] Test RTL layout on Firefox: verify consistent rendering with Chrome
- [ ] T049 [US4] Test RTL layout on Safari: verify consistent rendering
- [ ] T050 [US4] Test RTL layout on mobile Chrome/Safari: verify responsive RTL layout
- [ ] T051 [US4] Test code blocks remain LTR within RTL Urdu pages
- [ ] T052 [US4] Test mixed content (English technical terms in Urdu text) displays correctly

**Checkpoint**: At this point, User Stories 1, 2, AND 4 should all work - Urdu docs with proper RTL layout and language toggle

---

## Phase 6: User Story 3 - Interact with Chatbot in Urdu (Priority: P2)

**Goal**: Enable chatbot to detect Urdu queries and respond in Urdu

**Independent Test**: Ask chatbot questions in Urdu, verify Urdu responses; ask in English, verify English responses

### Implementation for User Story 3

- [ ] T053 [P] [US3] Create chatService.ts in humanoid-robotics-book/src/services/chatService.ts
- [ ] T054 [US3] Implement detectLanguage function using Unicode pattern matching (U+0600-U+06FF) in chatService.ts
- [ ] T055 [US3] Implement sendChatMessage function with language parameter in chatService.ts
- [ ] T056 [US3] Integrate chatService with existing chatbot UI component (identify and update component)
- [ ] T057 [US3] Update chatbot UI to call sendChatMessage with detected language
- [ ] T058 [US3] Update chatbot UI to respect current locale from useLanguage hook
- [ ] T059 [US3] Add error handling for translation service failures in chatService.ts
- [ ] T060 [US3] Add fallback to English responses if Urdu translation fails in chatService.ts
- [ ] T061 [US3] Test chatbot with Urdu query when toggle is set to Urdu
- [ ] T062 [US3] Test chatbot with English query when toggle is set to English
- [ ] T063 [US3] Test chatbot auto-detects Urdu query regardless of toggle setting
- [ ] T064 [US3] Test chatbot auto-detects English query regardless of toggle setting
- [ ] T065 [US3] Test chatbot language switching mid-conversation
- [ ] T066 [US3] Test chatbot error handling when translation service is unavailable
- [ ] T067 [US3] Test chatbot with mixed English/Urdu queries (edge case)

**Checkpoint**: All user stories should now be independently functional - full Urdu translation support complete

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final deployment

- [ ] T068 [P] Add Urdu font import (Noto Nastaliq Urdu) in humanoid-robotics-book/src/css/custom.css if needed
- [ ] T069 [P] Optimize translation file size: remove unused translation keys from i18n/ur/ files
- [ ] T070 [P] Add HTML lang attribute handling for Urdu pages (verify Docusaurus auto-handles this)
- [ ] T071 Perform full build for both locales: `npm run build` and check for warnings
- [ ] T072 Test language switching performance: measure time from toggle click to page re-render (< 2 seconds)
- [ ] T073 Test Urdu page load performance: run Lighthouse audit on /ur/ routes (target ≥ 90 score)
- [ ] T074 Verify all internal links work in Urdu version (check for broken /ur/ prefixed links)
- [ ] T075 Verify external links work correctly from Urdu pages
- [ ] T076 Test URL sharing: share /ur/ URL and verify it opens in Urdu
- [ ] T077 Test browser back/forward buttons with language switching
- [ ] T078 Test language toggle accessibility: keyboard navigation and ARIA labels
- [ ] T079 Run visual regression testing: compare EN and UR page layouts for consistency
- [ ] T080 Update README or developer docs with translation workflow instructions
- [ ] T081 Commit all changes with message: "feat: add Urdu translation support with language toggle and chatbot"
- [ ] T082 Push branch to remote: `git push origin 002-urdu-i18n-support`
- [ ] T083 Verify Vercel deployment succeeds and both / and /ur/ routes are accessible
- [ ] T084 Perform final end-to-end testing on deployed Vercel site

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Can start after Phase 2
- **User Story 2 (Phase 4)**: Depends on Foundational - Can start after Phase 2 (or in parallel with US1 if different developers)
- **User Story 4 (Phase 5)**: Depends on Foundational - Can start after Phase 2 (or in parallel with US1/US2)
- **User Story 3 (Phase 6)**: Depends on US2 (needs useLanguage hook) and US4 (needs RTL CSS for chat UI)
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories, but provides useLanguage hook for US3
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - No dependencies on other stories, enhances US1 and needed for US3
- **User Story 3 (P2)**: Depends on US2 (useLanguage hook) and US4 (RTL CSS) - Start after US2 and US4 complete

### Within Each User Story

- **US1**: All translation tasks can run in parallel (T012-T026 marked [P]), then build/test sequentially
- **US2**: Component and styles can be created in parallel (T031-T032 marked [P]), then integration sequentially
- **US4**: All RTL CSS additions can run in parallel (T042-T046 marked [P]), then testing sequentially
- **US3**: chatService creation parallel with other prep (T053 marked [P]), then integration sequentially

### Parallel Opportunities

- **Setup**: T003-T004 can run in parallel
- **Foundational**: T005-T008 can run in parallel (different JSON files)
- **US1**: T012-T026 can run in parallel (different markdown files - ideal for team collaboration)
- **US2**: T031-T032 can run in parallel
- **US4**: T042-T046 can run in parallel
- **US3**: T053 can start while other tasks in progress
- **Polish**: T068-T070 can run in parallel

---

## Parallel Example: User Story 1 (Documentation Translation)

```bash
# Launch all documentation translation tasks together (HIGHLY PARALLELIZABLE):
# Each task translates a different markdown file - perfect for team collaboration

Task: "Translate part1-foundations/intro.md"
Task: "Translate part1-foundations/chapter-01"
Task: "Translate part1-foundations/chapter-02"
Task: "Translate part1-foundations/chapter-03"
Task: "Translate part2-modules/module1-ros2/intro.md"
Task: "Translate part2-modules/module1-ros2/chapter-04"
Task: "Translate part2-modules/module1-ros2/chapter-05"
Task: "Translate part2-modules/module1-ros2/chapter-06"
Task: "Translate part2-modules/module1-ros2/chapter-07"
Task: "Translate part2-modules/module2-digital-twin sections"
Task: "Translate part2-modules/module3-isaac sections"
Task: "Translate part2-modules/module4-vla sections"
Task: "Translate part3-capstone sections"
Task: "Translate part4-future sections"

# With 4 translators, this could be completed in parallel:
# Translator 1: Part 1 (T013-T016)
# Translator 2: Module 1 (T017-T021)
# Translator 3: Modules 2-4 (T022-T024)
# Translator 4: Parts 3-4 (T025-T026)
```

---

## Parallel Example: User Story 2 (Language Toggle)

```bash
# Launch component creation tasks together:
Task: "Create LanguageToggle component in src/components/LanguageToggle/index.tsx"
Task: "Create LanguageToggle styles in src/components/LanguageToggle/styles.module.css"

# Once both complete, proceed with integration sequentially
```

---

## Parallel Example: User Story 4 (RTL CSS)

```bash
# Launch all RTL CSS additions together:
Task: "Add RTL-specific CSS for sidebar mirroring"
Task: "Add RTL-specific CSS for navbar alignment"
Task: "Add RTL-specific CSS for pagination button mirroring"
Task: "Add RTL-specific CSS for search box alignment"
Task: "Add CSS to keep URLs and links LTR within RTL pages"

# All modify the same file (rtl.css) but different sections - can be done in parallel if coordinated
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (view Urdu docs)
4. Complete Phase 4: User Story 2 (language toggle)
5. **STOP and VALIDATE**: Test US1 and US2 independently
6. Deploy/demo if ready - users can now read docs in Urdu and switch languages

### Incremental Delivery

1. **MVP Release**: Setup + Foundational + US1 + US2 → Basic Urdu docs with language toggle
2. **Enhancement 1**: Add US4 → Improved RTL layout and polish
3. **Enhancement 2**: Add US3 → Bilingual chatbot support
4. Each increment adds value without breaking previous functionality

### Parallel Team Strategy

With multiple developers:

1. **Week 1**: Team completes Setup (Phase 1) + Foundational (Phase 2) together
2. **Week 2**: Once Foundational is done:
   - Developer A: User Story 1 (doc translation - can delegate to translators)
   - Developer B: User Story 2 (language toggle component)
   - Developer C: User Story 4 (RTL CSS refinement)
3. **Week 3**:
   - Developer D: User Story 3 (chatbot integration) - starts after US2 complete
   - Others: Testing, polish, deployment
4. Stories integrate independently, minimal merge conflicts (different files)

---

## Notes

- [P] tasks = different files/sections, no dependencies - safe to parallelize
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Translation tasks (T012-T026) are highly parallelizable - ideal for team/translator collaboration
- RTL testing should be performed on multiple browsers per US4 requirements
- Chatbot integration (US3) requires external API - ensure backend endpoint is available
- Manual testing required per quickstart.md - no automated test suite
- Commit frequently after completing logical groups of tasks
- Verify Docusaurus build succeeds after each phase
- Stop at any checkpoint to validate story independently before proceeding

---

## Task Count Summary

- **Setup (Phase 1)**: 4 tasks
- **Foundational (Phase 2)**: 7 tasks
- **User Story 1 (Phase 3)**: 19 tasks (15 translation + 4 validation)
- **User Story 2 (Phase 4)**: 11 tasks
- **User Story 4 (Phase 5)**: 11 tasks
- **User Story 3 (Phase 6)**: 15 tasks
- **Polish (Phase 7)**: 17 tasks

**Total**: 84 tasks

**Parallel Opportunities**: 32 tasks marked [P] can run in parallel within their respective phases

**MVP Scope**: 41 tasks (Phase 1 + 2 + 3 + 4) - Delivers core Urdu documentation viewing and language switching

**Estimated Timeline**:
- MVP (US1 + US2): 2-3 weeks with translation team
- Full Feature (all stories): 3-4 weeks with translation team and testing
