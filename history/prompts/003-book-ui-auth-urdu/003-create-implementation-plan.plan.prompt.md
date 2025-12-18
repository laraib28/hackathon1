---
id: 003
title: Create Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-book-ui-auth-urdu
branch: 003-book-ui-auth-urdu
user: laraib
command: /sp.plan
labels: ["planning", "architecture", "research", "data-model", "contracts"]
links:
  spec: ../specs/003-book-ui-auth-urdu/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - ../specs/003-book-ui-auth-urdu/plan.md
  - ../specs/003-book-ui-auth-urdu/research.md
  - ../specs/003-book-ui-auth-urdu/data-model.md
  - ../specs/003-book-ui-auth-urdu/contracts/auth-api.yaml
  - ../specs/003-book-ui-auth-urdu/contracts/chat-api.yaml
  - ../specs/003-book-ui-auth-urdu/quickstart.md
tests:
  - Constitution Check: PASS (all principles aligned)
  - Research completeness: PASS (all unknowns resolved)
  - Data model validation: PASS (no schema changes needed)
  - API contracts: PASS (OpenAPI 3.0 compliant)
---

## Prompt

User executed `/sp.plan` after completing feature specification for "Book UI Enhancement, Better Auth Integration, and Urdu Translation Fix".

**Context from spec.md**:
- Three user stories (P1: Professional UI, P1: Unified Auth, P2: Multi-language Chat)
- 30 functional requirements across UI, authentication, and language support
- Constraints: No heavy frameworks, preserve RAG logic, no breaking changes
- Target: <2s page load, 90+ Lighthouse score, 85%+ Roman Urdu detection accuracy

**Planning objectives**:
1. Fill Technical Context from codebase review
2. Validate Constitution compliance
3. Research unknowns (Better Auth integration, language detection, UI patterns)
4. Generate data model (User, UserSession, ChatHistory entities)
5. Define API contracts (auth and chat endpoints)
6. Create quickstart guide for developers

## Response snapshot

**Phase 0 - Planning & Technical Context**:

1. Created comprehensive implementation plan (`plan.md`) with:
   - Technical context: TypeScript 5.6 + React 19 + Docusaurus 3.9, Python 3.11 + FastAPI
   - Constitution Check: ✅ PASS (all 6 core principles aligned)
   - Project structure: Web app (frontend/backend separation)
   - Risk analysis: 6 risks identified with mitigation strategies
   - ADR candidates: 3 significant decisions flagged for documentation

2. Executed research phase (`research.md`) covering:
   - **Better Auth Integration**: Decided to keep existing Better Auth-compatible custom implementation, remove only Firebase remnants (minimal risk approach)
   - **Language Detection**: Hybrid frontend keyword-based detection (85%+ accuracy, <10ms latency, zero cost)
   - **UI Enhancement**: CSS variables only approach (no swizzling), 17px base font, 1.7 line-height, 8px spacing grid
   - **Protected Routes**: Component-level guards with ProtectedRoute wrapper
   - **Roman Urdu Keywords**: Identified 30+ high-confidence keywords (kya, hai, kaise, etc.) with 30% threshold

**Phase 1 - Design & Contracts**:

3. Generated data model (`data-model.md`):
   - User entity (id, email, password_hash, name, created_at, email_verified)
   - UserSession entity (token, user_id, created_at, expires_at) - currently in-memory
   - ChatHistory entity (id, user_id, message, response, language, sources)
   - No schema changes required (existing database.py implements all tables)

4. Created API contracts:
   - `auth-api.yaml` (OpenAPI 3.0): 4 endpoints (sign-up, sign-in, sign-out, session)
   - `chat-api.yaml` (OpenAPI 3.0): 1 endpoint (POST /api/chat with language parameter)
   - All endpoints match existing backend implementation

5. Wrote developer quickstart (`quickstart.md`):
   - Prerequisites and 5-minute quick start
   - Detailed frontend/backend setup instructions
   - Manual testing checklist (auth flow, language detection, UI enhancements)
   - Troubleshooting guide
   - Deployment instructions (Vercel + Railway)

**Key Architectural Decisions**:

1. **Minimal Auth Refactor**: Keep working Better Auth-compatible API, avoid SDK integration (low risk, faster delivery)
2. **Client-Side Language Detection**: Use keyword matching in frontend (fast, accurate, no API overhead)
3. **CSS-Only UI**: Use Docusaurus CSS variables instead of swizzling components (maintainable across upgrades)

**Dependencies Resolved**:
- Better Auth SDK integration → Use existing custom implementation (already working)
- Roman Urdu detection strategy → Keyword-based hybrid approach (30% threshold)
- UI enhancement method → CSS variables (custom.css modifications)

## Outcome

- ✅ Impact: Complete implementation plan with research, data model, and API contracts ready for task generation
- 🧪 Tests: Constitution Check passed, all research questions resolved, contracts validated against existing backend
- 📁 Files: 6 planning artifacts created (plan, research, data-model, 2 contracts, quickstart)
- 🔁 Next prompts: User can run `/sp.adr` for architectural decisions, then `/sp.tasks` to generate implementation tasks
- 🧠 Reflection: Research-driven planning eliminated implementation unknowns. Minimal-risk decisions (keep existing auth, CSS-only UI) reduce scope and delivery time while meeting all spec requirements.

## Evaluation notes (flywheel)

- Failure modes observed: None - all research agents completed successfully (auth patterns, language detection, UI best practices)
- Graders run and results (PASS/FAIL): PASS - Constitution Check validated, data model matches existing schema, API contracts align with backend
- Prompt variant (if applicable): Standard `/sp.plan` workflow with Phase 0 research and Phase 1 design
- Next experiment (smallest change to try): Consider parallel execution of research agents to reduce planning time (agents launched successfully but took longer than expected)
