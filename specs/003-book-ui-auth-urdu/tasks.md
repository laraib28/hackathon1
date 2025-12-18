# Tasks: Book UI Enhancement, Better Auth Integration, and Urdu Translation Fix

**Input**: Design documents from `/specs/003-book-ui-auth-urdu/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Organization**: Tasks grouped by user story for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: User story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `humanoid-robotics-book/src/`
- **Backend**: `backend/`
- Web app structure with separate frontend/backend

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment configuration

- [ ] T001 Verify Node.js 20.x and Python 3.11+ installed, check package.json and requirements.txt
- [ ] T002 [P] Install frontend dependencies with `npm install` in humanoid-robotics-book/
- [ ] T003 [P] Install backend dependencies with `pip install -r requirements.txt` in backend/
- [ ] T004 [P] Create frontend environment file humanoid-robotics-book/.env.local with REACT_APP_API_URL
- [ ] T005 [P] Create backend environment file backend/.env with all required variables (DATABASE_URL, BETTER_AUTH_SECRET, OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY)
- [ ] T006 Initialize SQLite database with `python -c "from database import init_db; init_db()"` in backend/
- [ ] T007 [P] Run TypeScript type check with `npm run typecheck` to verify project compiles
- [ ] T008 [P] Start backend dev server with `uvicorn main_fastapi:app --reload` to verify it runs
- [ ] T009 [P] Start frontend dev server with `npm start` to verify Docusaurus loads

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure and utilities needed by ALL user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T010 Create language detection utility in humanoid-robotics-book/src/utils/languageDetector.ts with detectLanguage() function
- [ ] T011 Implement Urdu script detection (Unicode range \u0600-\u06FF) in languageDetector.ts
- [ ] T012 [P] Implement Roman Urdu keyword list (kya, hai, kaise, etc.) in languageDetector.ts
- [ ] T013 [P] Implement 30% threshold logic for Roman Urdu detection in languageDetector.ts
- [ ] T014 Add unit tests for languageDetector: English, Urdu script, Roman Urdu, mixed queries (optional)
- [ ] T015 Review existing AuthContext in humanoid-robotics-book/src/context/AuthContext.tsx and identify cleanup needs
- [ ] T016 Review existing authService in humanoid-robotics-book/src/services/authService.ts and verify Better Auth compatibility
- [ ] T017 [P] Search codebase for Firebase remnants: `grep -r "firebase" humanoid-robotics-book/src/` and list files to clean
- [ ] T018 [P] Search codebase for Firebase imports: `grep -r "from 'firebase" humanoid-robotics-book/src/` and note locations

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Reader Experiences Professional Book Interface (Priority: P1) 🎯 MVP

**Goal**: Deliver polished, professional reading experience with improved typography, spacing, and visual design

**Independent Test**: Load homepage and any book chapter page, evaluate typography (17px base, 1.7 line-height), spacing (2.5rem heading margins), hero section styling, and card design without needing authentication or chatbot

### Implementation for User Story 1

**Step 1: Typography System**

- [ ] T019 [P] [US1] Update base font size to 17px in humanoid-robotics-book/src/css/custom.css (--ifm-font-size-base)
- [ ] T020 [P] [US1] Update line height to 1.7 in humanoid-robotics-book/src/css/custom.css (--ifm-line-height-base)
- [ ] T021 [P] [US1] Update heading sizes (h1: 2.5rem, h2: 2rem, h3: 1.5rem) in humanoid-robotics-book/src/css/custom.css
- [ ] T022 [P] [US1] Update code font size to 0.9375rem in humanoid-robotics-book/src/css/custom.css (--ifm-code-font-size)
- [ ] T023 [P] [US1] Add heading margin styles (2.5rem top, 1.25rem bottom) in humanoid-robotics-book/src/css/custom.css for .markdown h1-h3

**Step 2: Spacing System**

- [ ] T024 [P] [US1] Update horizontal spacing to 1.5rem in humanoid-robotics-book/src/css/custom.css (--ifm-spacing-horizontal)
- [ ] T025 [P] [US1] Update vertical spacing to 1.5rem in humanoid-robotics-book/src/css/custom.css (--ifm-spacing-vertical)
- [ ] T026 [P] [US1] Add paragraph bottom margin (1.5rem) in humanoid-robotics-book/src/css/custom.css for .markdown p
- [ ] T027 [P] [US1] Add list spacing (margin-bottom: 1.5rem, padding-left: 2rem) in humanoid-robotics-book/src/css/custom.css for .markdown ul/ol

**Step 3: Homepage Hero Section**

- [ ] T028 [P] [US1] Add hero section styles in humanoid-robotics-book/src/css/custom.css (.hero class with 4rem padding, gradient background)
- [ ] T029 [P] [US1] Add hero title styles in humanoid-robotics-book/src/css/custom.css (.hero__title: 3rem font, 700 weight, -0.03em letter-spacing)
- [ ] T030 [P] [US1] Add hero subtitle styles in humanoid-robotics-book/src/css/custom.css (.hero__subtitle: 1.5rem font, max-width 600px, centered)
- [ ] T031 [US1] Update homepage content in humanoid-robotics-book/src/pages/index.tsx to use hero classes and improved copy

**Step 4: Card Component Styling**

- [ ] T032 [P] [US1] Add card styles in humanoid-robotics-book/src/css/custom.css (.card: 2rem padding, 8px border-radius, box-shadow, hover transform)
- [ ] T033 [US1] Update HomepageFeatures component in humanoid-robotics-book/src/components/HomepageFeatures/index.tsx to apply card classes

**Step 5: Navigation Improvements**

- [ ] T034 [P] [US1] Add navbar spacing in humanoid-robotics-book/src/css/custom.css (.navbar: 1rem padding, box-shadow)
- [ ] T035 [P] [US1] Add menu item spacing in humanoid-robotics-book/src/css/custom.css (.menu__list-item: 0.5rem margin-bottom)
- [ ] T036 [US1] Update ChapterControls component in humanoid-robotics-book/src/components/ChapterControls/ChapterControls.tsx for better navigation clarity

**Step 6: Responsive Design**

- [ ] T037 [P] [US1] Add mobile responsive styles in humanoid-robotics-book/src/css/custom.css (@media max-width 768px: reduce font sizes, adjust hero)
- [ ] T038 [US1] Test UI on mobile (375px), tablet (768px), and desktop (1440px) viewports in browser DevTools

**Step 7: Validation**

- [ ] T039 [US1] Run Docusaurus build with `npm run build` to verify no errors
- [ ] T040 [US1] Serve production build with `npm run serve` and visually inspect typography and spacing
- [ ] T041 [US1] Validate Lighthouse score remains 90+ for performance and accessibility
- [ ] T042 [US1] Test page load time <2s on throttled 3G network in Chrome DevTools

**Checkpoint**: At this point, book UI should look professional with improved typography, spacing, hero section, and navigation - fully testable without auth or chatbot

---

## Phase 4: User Story 2 - User Authenticates with Better Auth Only (Priority: P1)

**Goal**: Consolidate to single Better Auth authentication system, remove all legacy auth code, ensure protected routes work correctly

**Independent Test**: Attempt protected page access → redirects to login, signup new user → auto-login, login existing user → redirect to intended page, logout → clear session - all without needing UI improvements or translation

### Implementation for User Story 2

**Step 1: Legacy Auth Cleanup**

- [ ] T043 [P] [US2] Remove Firebase imports and config from all files identified in T017-T018 (if any)
- [ ] T044 [P] [US2] Remove EnhancedSignup.tsx if it's a duplicate of Signup.tsx in humanoid-robotics-book/src/components/Auth/
- [ ] T045 [P] [US2] Remove any custom localStorage-based session management code (check AuthContext.tsx for legacy patterns)
- [ ] T046 [US2] Simplify AuthContext in humanoid-robotics-book/src/context/AuthContext.tsx to use only authService.ts methods

**Step 2: Auth Service Verification**

- [ ] T047 [US2] Review authService.ts in humanoid-robotics-book/src/services/authService.ts and verify it correctly calls /api/auth/sign-up/email, /api/auth/sign-in/email, /api/auth/session
- [ ] T048 [US2] Add error handling to authService.ts for network failures and 401 responses
- [ ] T049 [US2] Verify authService.ts stores session token in localStorage as 'authToken' after signup/login
- [ ] T050 [US2] Verify authService.ts sends token as `Authorization: Bearer <token>` header in getSession()

**Step 3: Authentication Components**

- [ ] T051 [P] [US2] Update Login component in humanoid-robotics-book/src/components/Auth/Login.tsx to use authService and handle redirects
- [ ] T052 [P] [US2] Update Signup component in humanoid-robotics-book/src/components/Auth/Signup.tsx to use authService and auto-login after registration
- [ ] T053 [P] [US2] Update UserProfile component in humanoid-robotics-book/src/components/Auth/UserProfile.tsx to fetch user data from authService.getSession()
- [ ] T054 [US2] Implement logout functionality in UserProfile or navbar to call authService.signOut() and clear localStorage

**Step 4: Protected Routes**

- [ ] T055 [US2] Update ProtectedRoute component in humanoid-robotics-book/src/components/Auth/ProtectedRoute.tsx to check authService.getSession() and redirect unauthenticated users to `/login?redirect=<current-path>`
- [ ] T056 [US2] Wrap profile page in humanoid-robotics-book/src/pages/profile.tsx with ProtectedRoute component
- [ ] T057 [US2] Verify homepage, book content pages, and chatbot remain publicly accessible (no ProtectedRoute wrapper)

**Step 5: Auth Pages**

- [ ] T058 [P] [US2] Update login page in humanoid-robotics-book/src/pages/login.tsx to render Login component and handle redirect parameter
- [ ] T059 [P] [US2] Update signup page in humanoid-robotics-book/src/pages/signup.tsx to render Signup component and redirect after auto-login
- [ ] T060 [US2] Add redirect logic: if already authenticated on /login or /signup, redirect to homepage

**Step 6: Backend Validation**

- [ ] T061 [US2] Test backend auth endpoints manually with curl or Postman: POST /api/auth/sign-up/email, POST /api/auth/sign-in/email, GET /api/auth/session, POST /api/auth/sign-out
- [ ] T062 [US2] Verify backend returns correct response format: {user: {id, email, name, emailVerified, createdAt}, session: {token, expiresAt}}
- [ ] T063 [US2] Verify backend validates session tokens correctly and returns 401 for invalid/expired tokens

**Step 7: Integration Testing**

- [ ] T064 [US2] Test signup flow: Navigate to /signup, enter email/password/name, submit, verify auto-login and redirect to homepage
- [ ] T065 [US2] Test login flow: Navigate to /login, enter valid credentials, submit, verify redirect to homepage or intended page
- [ ] T066 [US2] Test protected route: Navigate to /profile without login, verify redirect to /login?redirect=/profile, login, verify redirect back to /profile
- [ ] T067 [US2] Test session persistence: Login, refresh page, verify still authenticated without re-login
- [ ] T068 [US2] Test logout: Click logout button, verify authToken removed from localStorage and redirected to homepage
- [ ] T069 [US2] Test invalid credentials: Login with wrong password, verify error message displayed
- [ ] T070 [US2] Test duplicate signup: Signup with existing email, verify "Email already registered" error

**Step 8: Codebase Verification**

- [ ] T071 [US2] Run grep search to confirm no Firebase code remains: `grep -r "firebase\|getAuth\|initializeApp" humanoid-robotics-book/src/` should return no results
- [ ] T072 [US2] Run grep search to confirm only Better Auth-compatible patterns exist: `grep -r "authService\|Better Auth" humanoid-robotics-book/src/` should show only authService usage
- [ ] T073 [US2] Review all auth-related files and confirm single authentication flow with no duplicates or legacy code

**Checkpoint**: At this point, authentication should use only Better Auth-compatible implementation, all protected routes should work, and zero legacy auth code should remain

---

## Phase 5: User Story 3 - Reader Asks Questions in Urdu or Roman Urdu (Priority: P2)

**Goal**: Enable chatbot to detect Urdu (script and Roman) and respond in appropriate language using OpenAI translation

**Independent Test**: Send English query → English response, Urdu script query → Urdu response, Roman Urdu query ("Robotics kya hai?") → Urdu response, verify 85%+ Roman Urdu detection accuracy - all without needing UI or auth changes

### Implementation for User Story 3

**Step 1: Language Detection Integration**

- [ ] T074 [US3] Import languageDetector utility in humanoid-robotics-book/src/components/ChatWidget/ChatWidget.tsx
- [ ] T075 [US3] Call detectLanguage(message) before sending chat request in ChatWidget.tsx and store result as target_language
- [ ] T076 [US3] Pass target_language parameter to backend API in chat request payload in ChatWidget.tsx

**Step 2: Chat Widget Updates**

- [ ] T077 [P] [US3] Add language state management to ChatWidget.tsx to track detected language for each message
- [ ] T078 [P] [US3] Update chat message rendering in ChatWidget.tsx to apply RTL text direction for Urdu responses
- [ ] T079 [US3] Verify ChatWidget.tsx UI remains unchanged (only backend integration changes, no visual redesign)

**Step 3: Backend Language Validation**

- [ ] T080 [US3] Review chat_api.py in backend/chat_api.py and verify it accepts target_language parameter ('en' or 'ur')
- [ ] T081 [US3] Verify backend uses correct OpenAI system prompt for Urdu translation when target_language='ur' in backend/chat_api.py
- [ ] T082 [US3] Verify backend fallback messages are in correct language (English or Urdu) based on target_language in backend/chat_api.py

**Step 4: Urdu Font Support**

- [ ] T083 [P] [US3] Verify Urdu font support in humanoid-robotics-book/src/css/rtl.css (Noto Nastaliq Urdu or system Arabic font)
- [ ] T084 [P] [US3] Add conditional font loading for Urdu if not already present in rtl.css
- [ ] T085 [US3] Test Urdu text rendering in chatbot to ensure Arabic script displays correctly

**Step 5: Testing Language Detection**

- [ ] T086 [US3] Test English query: "What is a humanoid robot?" → Verify target_language='en' sent to backend, English response received
- [ ] T087 [US3] Test Urdu script query: "روبوٹکس کیا ہے؟" → Verify target_language='ur' sent to backend, Urdu response received
- [ ] T088 [US3] Test Roman Urdu query: "Robotics kya hai?" → Verify target_language='ur' sent to backend, Urdu response received
- [ ] T089 [US3] Test Roman Urdu query: "Humanoid robot kaise kaam karta hai?" → Verify detection accuracy for multi-word Roman Urdu
- [ ] T090 [US3] Test edge case: "What is kya?" → Verify detected as 'en' (only 1 Urdu word, <30% threshold)
- [ ] T091 [US3] Test fallback: Ask about non-book topic in Urdu → Verify fallback message in Urdu "معاف کیجیے، فراہم کردہ مواد میں..."
- [ ] T092 [US3] Test fallback: Ask about non-book topic in English → Verify fallback message in English "I don't have that information..."

**Step 6: Response Quality Validation**

- [ ] T093 [US3] Test Urdu response quality: Ask 5 different robotics questions in Roman Urdu, verify responses are natural Urdu without unnecessary English mixing
- [ ] T094 [US3] Verify RAG sources are returned correctly for all languages (English and Urdu queries both include source URLs)
- [ ] T095 [US3] Test conversation history: Send multi-turn conversation in Urdu, verify context is maintained across messages

**Step 7: Performance & Accuracy**

- [ ] T096 [US3] Measure language detection latency in browser DevTools: should be <10ms for client-side detection
- [ ] T097 [US3] Test Roman Urdu detection accuracy with 20 sample queries: target 85%+ correct detection rate
- [ ] T098 [US3] Verify chatbot response time remains acceptable (<3s total including RAG + OpenAI) for both English and Urdu

**Checkpoint**: At this point, chatbot should correctly detect English, Urdu script, and Roman Urdu queries, and respond in appropriate language with 85%+ accuracy

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements affecting multiple user stories

- [ ] T099 [P] Run full TypeScript type check with `npm run typecheck` and fix any type errors
- [ ] T100 [P] Run Docusaurus build with `npm run build` and fix any build warnings or errors
- [ ] T101 [P] Test complete user journey: Homepage → Read chapter → Open chatbot (Urdu query) → Signup → Protected page → Logout
- [ ] T102 [P] Verify all environment variables documented in quickstart.md match actual .env files
- [ ] T103 [P] Test on multiple browsers: Chrome, Firefox, Safari, Edge for compatibility
- [ ] T104 [P] Run Lighthouse audit and verify scores: Performance 90+, Accessibility 90+, Best Practices 90+, SEO 90+
- [ ] T105 Add ADR for authentication consolidation decision (if user requests): document Better Auth choice, alternatives considered, rationale
- [ ] T106 Add ADR for language detection strategy (if user requests): document keyword-based approach, alternatives, accuracy trade-offs
- [ ] T107 Add ADR for UI enhancement approach (if user requests): document CSS-only decision, swizzling rejected, maintainability rationale
- [ ] T108 [P] Update quickstart.md if any setup steps changed during implementation
- [ ] T109 [P] Clean up console.log statements and debug code from all components
- [ ] T110 Final manual test checklist from quickstart.md: Auth flow, language detection, UI enhancements

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T001-T009) - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (T010-T018) - No dependencies on other stories
- **User Story 2 (Phase 4)**: Depends on Foundational (T010-T018) - No dependencies on other stories
- **User Story 3 (Phase 5)**: Depends on Foundational (T010-T018) - No dependencies on other stories
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - UI)**: Can start after Foundational → **INDEPENDENT** (no auth or chat needed)
- **User Story 2 (P1 - Auth)**: Can start after Foundational → **INDEPENDENT** (no UI or chat needed)
- **User Story 3 (P2 - Language)**: Can start after Foundational → **INDEPENDENT** (uses existing chatbot UI, no auth changes)

**🎯 All three user stories are independently testable and can be developed in parallel after Foundational phase**

### Within Each User Story

**User Story 1 (UI)**:
- Typography tasks (T019-T023) can run in parallel
- Spacing tasks (T024-T027) can run in parallel
- Hero section tasks (T028-T031) must run sequentially (CSS then React component)
- Card styling (T032-T033) must run sequentially (CSS then component)
- Validation tasks (T039-T042) run sequentially at end

**User Story 2 (Auth)**:
- Legacy cleanup tasks (T043-T046) can run in parallel
- Auth components (T051-T053) can run in parallel
- Auth pages (T058-T059) can run in parallel
- Integration tests (T064-T070) run sequentially

**User Story 3 (Language)**:
- Font support tasks (T083-T084) can run in parallel
- Test queries (T086-T092) can run in parallel
- Performance tests (T096-T098) can run in parallel

### Parallel Opportunities

**Setup Phase (9 tasks, 7 parallelizable)**:
- T002, T003, T004, T005, T007, T008, T009 can all run in parallel

**Foundational Phase (9 tasks, 4 parallelizable)**:
- T012, T013, T017, T018 can run in parallel

**User Story 1 (24 tasks, 17 parallelizable)**:
- Typography: T019-T023 (5 parallel)
- Spacing: T024-T027 (4 parallel)
- Hero: T028-T030 (3 parallel)
- Cards: T032 (1 parallel)
- Navigation: T034-T035 (2 parallel)
- Responsive: T037 (1 parallel)

**User Story 2 (31 tasks, 11 parallelizable)**:
- Cleanup: T043-T045 (3 parallel)
- Components: T051-T053 (3 parallel)
- Pages: T058-T059 (2 parallel)
- Backend: T061 (1 parallel)
- Verification: T071-T072 (2 parallel)

**User Story 3 (25 tasks, 7 parallelizable)**:
- Widget: T077-T078 (2 parallel)
- Fonts: T083-T084 (2 parallel)
- Tests: T086-T092 can be run as parallel test suite
- Polish: T099-T104, T108-T109 (6 parallel)

**Total**: 110 tasks (45 parallelizable within phases)

---

## Parallel Example: User Story 1 (UI Enhancement)

```bash
# Typography tasks - launch together:
Task T019: "Update base font size to 17px in custom.css"
Task T020: "Update line height to 1.7 in custom.css"
Task T021: "Update heading sizes in custom.css"
Task T022: "Update code font size in custom.css"
Task T023: "Add heading margin styles in custom.css"

# Spacing tasks - launch together:
Task T024: "Update horizontal spacing in custom.css"
Task T025: "Update vertical spacing in custom.css"
Task T026: "Add paragraph margin in custom.css"
Task T027: "Add list spacing in custom.css"

# Hero section styles - launch together:
Task T028: "Add hero section styles in custom.css"
Task T029: "Add hero title styles in custom.css"
Task T030: "Add hero subtitle styles in custom.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Professional UI)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: Foundational (T010-T018)
3. Complete Phase 3: User Story 1 (T019-T042)
4. **STOP and VALIDATE**: Test UI independently → homepage, book chapters, typography, spacing, responsiveness
5. Deploy/demo professional book interface

**Estimated MVP delivery**: ~15 tasks (Setup) + ~10 tasks (Foundational) + ~24 tasks (US1) = **49 tasks**

### Incremental Delivery

1. **Foundation** (T001-T018) → Environment ready, language detection utility, legacy auth identified
2. **+ User Story 1** (T019-T042) → Professional UI launched → **MVP Demo**
3. **+ User Story 2** (T043-T073) → Unified authentication → **V1.1 Demo**
4. **+ User Story 3** (T074-T098) → Multi-language support → **V1.2 Demo**
5. **+ Polish** (T099-T110) → Production-ready → **V2.0 Release**

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With 3 developers after Foundational phase:

1. **Team completes Setup + Foundational together** (T001-T018)
2. **Once Foundational is done**:
   - **Developer A**: User Story 1 (UI) → T019-T042
   - **Developer B**: User Story 2 (Auth) → T043-T073
   - **Developer C**: User Story 3 (Language) → T074-T098
3. Stories complete independently and integrate without conflicts
4. Team reconvenes for Polish phase (T099-T110)

**Timeline estimate**:
- Setup: 1-2 hours
- Foundational: 2-3 hours
- User Stories (parallel): 1 day per story (3 days total if sequential, 1 day if parallel with 3 devs)
- Polish: 2-4 hours
- **Total**: 2-4 days (sequential) or 1-2 days (parallel with team)

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label (US1, US2, US3) maps task to specific user story for traceability
- Each user story is independently completable and testable
- Commit after each task or logical group of parallel tasks
- Stop at any checkpoint to validate story independently before proceeding
- User Story 1 (UI) is the recommended MVP - delivers immediate visual value
- User Story 2 (Auth) and User Story 3 (Language) can be added incrementally
- All tasks include specific file paths for clarity and immediate executability

---

## Validation Checklist

**Format Compliance**:
- ✅ All tasks use `- [ ] [ID] [P?] [Story?] Description` format
- ✅ Task IDs sequential (T001-T110)
- ✅ [P] marker only on parallelizable tasks
- ✅ [Story] label (US1, US2, US3) on all user story tasks
- ✅ File paths included in all implementation tasks

**Organization**:
- ✅ Tasks grouped by user story (Phase 3, 4, 5)
- ✅ Each user story independently testable
- ✅ Clear checkpoints after each story
- ✅ Dependencies documented

**Completeness**:
- ✅ All functional requirements from spec.md covered
- ✅ All entities from data-model.md addressed (no schema changes needed)
- ✅ All endpoints from contracts/ validated
- ✅ Research decisions from research.md implemented

**Total Task Count**: 110 tasks
- Setup: 9 tasks
- Foundational: 9 tasks
- User Story 1 (UI): 24 tasks
- User Story 2 (Auth): 31 tasks
- User Story 3 (Language): 25 tasks
- Polish: 12 tasks

**Parallel Opportunities**: 45 tasks marked [P] (41% parallelization rate)

**Suggested MVP Scope**: Phase 1 + Phase 2 + Phase 3 (User Story 1 - Professional UI) = 42 tasks
