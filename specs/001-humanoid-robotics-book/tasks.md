# Tasks: Humanoid Robotics & Physical AI Book

**Input**: Design documents from `/specs/001-humanoid-robotics-book/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Tests**: This is an educational content project. No automated code tests are applicable. Validation comes from quality checks and content review.

**Organization**: Tasks are grouped by user story (book sections) to enable independent writing and validation of each part/module.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different chapters, no dependencies)
- **[Story]**: Which user story this task belongs to (US1-US7)
- Include exact file paths in descriptions

## Path Conventions

- **Book content**: `docs/` directory (Docusaurus structure)
- **Static assets**: `static/img/` directory
- **Documentation**: `specs/001-humanoid-robotics-book/` directory
- **Project root**: Docusaurus configuration files

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize Docusaurus project and establish book structure

- [ ] T001 Initialize Docusaurus 3.x project with Node.js 20.x LTS dependencies
- [ ] T002 [P] Configure docusaurus.config.js with site title, theme, and GitHub Pages deployment settings
- [ ] T003 [P] Create directory structure: docs/part1-foundations/, docs/part2-modules/, docs/part3-capstone/, docs/part4-future/
- [ ] T004 [P] Configure sidebars.js for hierarchical navigation (Parts → Modules → Chapters)
- [ ] T005 [P] Set up static asset directories: static/img/part1-foundations/, static/img/part2-modules/, static/img/part3-capstone/, static/img/part4-future/
- [ ] T006 [P] Install Mermaid plugin for diagrams-as-code (@docusaurus/theme-mermaid)
- [ ] T007 [P] Create .gitignore for Node.js and Docusaurus (node_modules/, build/, .docusaurus/)
- [ ] T008 Verify local build succeeds with `npm run build` (zero errors/warnings)

---

## Phase 2: Foundational (Research & Architecture)

**Purpose**: Core research and structural decisions that ALL book content depends on

**⚠️ CRITICAL**: No chapter writing can begin until this phase is complete

- [ ] T009 Create specs/001-humanoid-robotics-book/research.md documenting ROS 2, Gazebo, Unity, Isaac, VLA feasibility validation
- [ ] T010 [P] Gather ROS 2 official documentation sources (https://docs.ros.org/en/humble/) and add to research.md
- [ ] T011 [P] Gather NVIDIA Isaac documentation sources (https://docs.omniverse.nvidia.com/isaacsim/latest/) and add to research.md
- [ ] T012 [P] Gather Gazebo documentation sources (https://gazebosim.org/docs) and add to research.md
- [ ] T013 [P] Gather Unity Robotics Hub sources and add to research.md
- [ ] T014 [P] Gather Whisper and VLA literature sources (GitHub, papers) and add to research.md
- [ ] T015 Create specs/001-humanoid-robotics-book/sources.md with APA-formatted bibliography of all primary sources
- [ ] T016 Validate ROS 2 + Isaac integration feasibility and document in research.md
- [ ] T017 Validate VLA pipeline (Whisper → LLM → ROS 2 action) architectural soundness and document in research.md
- [ ] T018 Create 24 chapter skeleton files with title, learning objectives placeholder, prerequisites placeholder, section headings
- [ ] T019 [P] Create ADR-001: Decision on technical detail depth (conceptual + pseudocode) in history/adr/0001-technical-detail-depth.md
- [ ] T020 [P] Create ADR-002: Decision on conceptual vs. practical examples (hybrid progressive) in history/adr/0002-example-complexity.md
- [ ] T021 [P] Create ADR-003: Decision on diagram types (Mermaid primary, SVG complex) in history/adr/0003-diagram-formats.md
- [ ] T022 [P] Create ADR-004: Decision on tone and accessibility (professional/approachable) in history/adr/0004-tone-accessibility.md
- [ ] T023 Create specs/001-humanoid-robotics-book/data-model.md documenting book entities (Part, Module, Chapter, Code Example, Learning Objective, Diagram, Prerequisite, Career Path)
- [ ] T024 Create specs/001-humanoid-robotics-book/quickstart.md with reader onboarding guide (prerequisites, how to navigate, hands-on options)

**Checkpoint**: Foundation ready - chapter writing can now begin in parallel by module

---

## Phase 3: User Story 1 - Foundational Understanding (Priority: P1) 🎯 MVP

**Goal**: Deliver Part 1 (Chapters 1-3) teaching Physical AI, humanoid robotics basics, and why embodied intelligence matters

**Independent Test**: Reader can explain Physical AI concepts, describe humanoid robot characteristics, and articulate why embodied intelligence matters

### Implementation for User Story 1

- [ ] T025 [P] [US1] Write Chapter 1 "What is Physical AI?" in docs/part1-foundations/chapter01-what-is-physical-ai.md (2000-4000 words, FK 10-12, APA citations)
- [ ] T026 [P] [US1] Write Chapter 2 "Understanding Humanoid Robots" in docs/part1-foundations/chapter02-understanding-humanoid-robots.md (2000-4000 words, FK 10-12, APA citations)
- [ ] T027 [P] [US1] Write Chapter 3 "Why Embodied Intelligence Matters" in docs/part1-foundations/chapter03-why-embodied-intelligence-matters.md (2000-4000 words, FK 10-12, APA citations)
- [ ] T028 [P] [US1] Create Mermaid diagram "Physical AI vs Traditional AI" for Chapter 1
- [ ] T029 [P] [US1] Create Mermaid diagram "Humanoid Robot Components" for Chapter 2
- [ ] T030 [P] [US1] Create Mermaid diagram "Embodied Intelligence Applications" for Chapter 3
- [ ] T031 [US1] Run Flesch-Kincaid readability check on all Part 1 chapters (target: grade 10-12)
- [ ] T032 [US1] Validate all Part 1 factual claims against primary sources from sources.md
- [ ] T033 [US1] Check Part 1 for broken internal/external links
- [ ] T034 [US1] Verify all diagrams have alt text and figure captions
- [ ] T035 [US1] Test local Docusaurus build with Part 1 content (zero errors/warnings)

**Checkpoint**: Part 1 complete and independently testable - readers can understand foundational concepts

---

## Phase 4: User Story 2 - ROS 2 Communication Mastery (Priority: P2)

**Goal**: Deliver Module 1 (Chapters 4-7) teaching ROS 2 architecture, nodes, topics, services, actions, rclpy, and URDF modeling

**Independent Test**: Reader can diagram ROS 2 communication flow from AI agent command to robot action

### Implementation for User Story 2

- [ ] T036 [P] [US2] Write Chapter 4 "Introduction to ROS 2 for Humanoid Robots" in docs/part2-modules/module1-ros2/chapter04-intro-ros2.md (2000-4000 words)
- [ ] T037 [P] [US2] Write Chapter 5 "Nodes, Topics, Services, and Actions" in docs/part2-modules/module1-ros2/chapter05-nodes-topics-services.md (2000-4000 words)
- [ ] T038 [P] [US2] Write Chapter 6 "Bridging Python Agents to ROS Using rclpy" in docs/part2-modules/module1-ros2/chapter06-python-agents-rclpy.md (2000-4000 words)
- [ ] T039 [P] [US2] Write Chapter 7 "URDF: Modeling a Humanoid in Software" in docs/part2-modules/module1-ros2/chapter07-urdf-modeling.md (2000-4000 words)
- [ ] T040 [P] [US2] Create Mermaid sequence diagram "ROS 2 Publish-Subscribe Pattern" for Chapter 4
- [ ] T041 [P] [US2] Create Mermaid diagram "ROS 2 Node Communication Topology" for Chapter 5
- [ ] T042 [P] [US2] Create conceptual Python pseudocode "rclpy Action Client Flow" for Chapter 6
- [ ] T043 [P] [US2] Create Mermaid diagram "URDF Structure (Links, Joints, Sensors)" for Chapter 7
- [ ] T044 [US2] Run Flesch-Kincaid readability check on all Module 1 chapters (target: grade 10-12)
- [ ] T045 [US2] Validate all Module 1 ROS 2 claims against official ROS 2 documentation
- [ ] T046 [US2] Verify Module 1 terminology matches ROS 2 official docs (e.g., "action client" not "action requester")
- [ ] T047 [US2] Check Module 1 for broken links and validate version-specific info (ROS 2 Humble)
- [ ] T048 [US2] Test local Docusaurus build with Part 1 + Module 1 content

**Checkpoint**: Module 1 complete - readers understand ROS 2 communication backbone

---

## Phase 5: User Story 3 - Digital Twin Simulation (Priority: P3)

**Goal**: Deliver Module 2 (Chapters 8-11) teaching digital twin concepts, Gazebo physics, sensor simulation, and Unity visualization

**Independent Test**: Reader can explain digital twin purpose, physics simulation, and identify sensors needed for humanoid tasks

### Implementation for User Story 3

- [ ] T049 [P] [US3] Write Chapter 8 "What is a Digital Twin in Robotics?" in docs/part2-modules/module2-digital-twin/chapter08-what-is-digital-twin.md (2000-4000 words)
- [ ] T050 [P] [US3] Write Chapter 9 "Gazebo Simulation: Physics, Collisions, and Balance" in docs/part2-modules/module2-digital-twin/chapter09-gazebo-simulation.md (2000-4000 words)
- [ ] T051 [P] [US3] Write Chapter 10 "Simulating Sensors (LiDAR, Depth, IMU)" in docs/part2-modules/module2-digital-twin/chapter10-simulating-sensors.md (2000-4000 words)
- [ ] T052 [P] [US3] Write Chapter 11 "Unity Visualization for Humanoid Interaction" in docs/part2-modules/module2-digital-twin/chapter11-unity-visualization.md (2000-4000 words)
- [ ] T053 [P] [US3] Create Mermaid diagram "Digital Twin Workflow (Design → Simulate → Deploy)" for Chapter 8
- [ ] T054 [P] [US3] Create Mermaid diagram "Gazebo Physics Engine Components" for Chapter 9
- [ ] T055 [P] [US3] Create diagram "Sensor Simulation Pipeline" for Chapter 10
- [ ] T056 [P] [US3] Create comparison diagram "Gazebo vs Unity Use Cases" for Chapter 11
- [ ] T057 [US3] Run Flesch-Kincaid readability check on all Module 2 chapters
- [ ] T058 [US3] Validate all Gazebo/Unity claims against official documentation
- [ ] T059 [US3] Verify Module 2 references Module 1 concepts correctly (URDF from Ch 7)
- [ ] T060 [US3] Check Module 2 for broken links
- [ ] T061 [US3] Test local Docusaurus build with Part 1 + Modules 1-2

**Checkpoint**: Module 2 complete - readers understand simulation environments

---

## Phase 6: User Story 4 - NVIDIA Isaac AI Integration (Priority: P4)

**Goal**: Deliver Module 3 (Chapters 12-15) teaching Isaac Sim, perception (VSLAM, object detection), navigation (Nav2), and RL/sim-to-real

**Independent Test**: Reader can describe Isaac Sim synthetic data generation, visual SLAM, and explain sim-to-real gap

### Implementation for User Story 4

- [ ] T062 [P] [US4] Write Chapter 12 "NVIDIA Isaac Sim Overview" in docs/part2-modules/module3-isaac/chapter12-isaac-sim-overview.md (2000-4000 words)
- [ ] T063 [P] [US4] Write Chapter 13 "Isaac ROS Perception (VSLAM, Depth, Object Detection)" in docs/part2-modules/module3-isaac/chapter13-isaac-ros-perception.md (2000-4000 words)
- [ ] T064 [P] [US4] Write Chapter 14 "Navigation & Path Planning (Nav2)" in docs/part2-modules/module3-isaac/chapter14-navigation-path-planning.md (2000-4000 words)
- [ ] T065 [P] [US4] Write Chapter 15 "Training Humanoid Skills (Reinforcement Learning & Sim-to-Real)" in docs/part2-modules/module3-isaac/chapter15-training-humanoid-skills.md (2000-4000 words)
- [ ] T066 [P] [US4] Create Mermaid diagram "Isaac Sim Architecture" for Chapter 12
- [ ] T067 [P] [US4] Create Mermaid sequence diagram "Visual SLAM Pipeline" for Chapter 13
- [ ] T068 [P] [US4] Create flowchart "Nav2 Path Planning Flow" for Chapter 14
- [ ] T069 [P] [US4] Create diagram "RL Training Loop + Sim-to-Real Transfer" for Chapter 15
- [ ] T070 [US4] Run Flesch-Kincaid readability check on all Module 3 chapters
- [ ] T071 [US4] Validate all Isaac claims against NVIDIA Isaac official documentation
- [ ] T072 [US4] Verify Module 3 references Modules 1-2 correctly (ROS 2 from M1, simulation from M2)
- [ ] T073 [US4] Check Module 3 for broken links and validate Isaac version info
- [ ] T074 [US4] Test local Docusaurus build with Part 1 + Modules 1-3

**Checkpoint**: Module 3 complete - readers understand Isaac AI perception and navigation

---

## Phase 7: User Story 5 - Vision-Language-Action Pipeline (Priority: P5)

**Goal**: Deliver Module 4 (Chapters 16-19) teaching VLA architecture, Whisper, LLM planning, and capstone pipeline integration

**Independent Test**: Reader can trace complete VLA pipeline from voice input through perception to action execution

### Implementation for User Story 5

- [ ] T075 [P] [US5] Write Chapter 16 "What is VLA? (Vision + Language + Action)" in docs/part2-modules/module4-vla/chapter16-what-is-vla.md (2000-4000 words)
- [ ] T076 [P] [US5] Write Chapter 17 "Voice-to-Action (Whisper → LLM → Plan)" in docs/part2-modules/module4-vla/chapter17-voice-to-action.md (2000-4000 words)
- [ ] T077 [P] [US5] Write Chapter 18 "How LLMs Generate Robot Actions" in docs/part2-modules/module4-vla/chapter18-llms-generate-actions.md (2000-4000 words)
- [ ] T078 [US5] Write Chapter 19 "Capstone Implementation: The Autonomous Humanoid" in docs/part2-modules/module4-vla/chapter19-capstone-implementation.md (2000-4000 words, integrates all modules)
- [ ] T079 [P] [US5] Create Mermaid diagram "VLA Architecture Overview" for Chapter 16
- [ ] T080 [P] [US5] Create Mermaid sequence diagram "Voice → Whisper → LLM → ROS 2 Flow" for Chapter 17
- [ ] T081 [P] [US5] Create flowchart "LLM Task Decomposition Process" for Chapter 18
- [ ] T082 [US5] Create comprehensive SVG diagram "Complete Autonomous Humanoid Pipeline (All Modules)" for Chapter 19
- [ ] T083 [US5] Run Flesch-Kincaid readability check on all Module 4 chapters
- [ ] T084 [US5] Validate all VLA claims against Whisper GitHub and LLM robotics literature
- [ ] T085 [US5] Verify Chapter 19 capstone correctly references all four modules (ROS 2, Digital Twin, Isaac, VLA)
- [ ] T086 [US5] Check Module 4 for broken links
- [ ] T087 [US5] Test local Docusaurus build with Part 1 + all 4 modules

**Checkpoint**: All four modules complete - readers understand complete VLA-enabled humanoid system

---

## Phase 8: User Story 6 - Complete System Integration (Priority: P6)

**Goal**: Deliver Part 3 (Chapters 20-22) showing complete autonomous humanoid architecture and integration patterns

**Independent Test**: Reader can design own humanoid system by selecting appropriate components for each layer

### Implementation for User Story 6

- [ ] T088 [US6] Write Chapter 20 "Complete Architecture Overview" in docs/part3-capstone/chapter20-complete-architecture.md (2000-4000 words)
- [ ] T089 [US6] Write Chapter 21 "Integrating All Modules Into One Pipeline" in docs/part3-capstone/chapter21-integrating-all-modules.md (2000-4000 words)
- [ ] T090 [US6] Write Chapter 22 "Final Simulation Walkthrough" in docs/part3-capstone/chapter22-final-simulation-walkthrough.md (2000-4000 words)
- [ ] T091 [US6] Create comprehensive SVG diagram "Full System Architecture (Layers: Communication, Simulation, Perception, Cognition)" for Chapter 20
- [ ] T092 [P] [US6] Create Mermaid sequence diagram "End-to-End Task Execution Flow" for Chapter 21
- [ ] T093 [P] [US6] Create walkthrough diagram "Humanoid Completing Hotel Concierge Task" for Chapter 22
- [ ] T094 [US6] Validate backward tracing: every capstone concept introduced in earlier chapters
- [ ] T095 [US6] Create dependency matrix showing which modules contribute to which capstone components
- [ ] T096 [US6] Run Flesch-Kincaid readability check on all Part 3 chapters
- [ ] T097 [US6] Check Part 3 for broken links and cross-references to all modules
- [ ] T098 [US6] Test local Docusaurus build with Parts 1-3 complete

**Checkpoint**: Capstone complete - readers understand complete system integration

---

## Phase 9: User Story 7 - Career and Future Exploration (Priority: P7)

**Goal**: Deliver Part 4 (Chapters 23-24) covering future trends and career opportunities in humanoid robotics

**Independent Test**: Reader can identify career paths, emerging technologies, and create personal learning roadmap

### Implementation for User Story 7

- [ ] T099 [P] [US7] Write Chapter 23 "The Next Decade of Embodied AI" in docs/part4-future/chapter23-next-decade-embodied-ai.md (2000-4000 words)
- [ ] T100 [P] [US7] Write Chapter 24 "Opportunities, Careers, and What's Next" in docs/part4-future/chapter24-opportunities-careers.md (2000-4000 words)
- [ ] T101 [P] [US7] Create diagram "Emerging Trends in Humanoid Robotics (2025-2035)" for Chapter 23
- [ ] T102 [P] [US7] Create diagram "Career Paths in Embodied AI" for Chapter 24
- [ ] T103 [US7] Run Flesch-Kincaid readability check on Part 4 chapters
- [ ] T104 [US7] Validate future trend claims against recent conference talks (ROSCon, GTC) and papers
- [ ] T105 [US7] Check Part 4 for broken links
- [ ] T106 [US7] Test local Docusaurus build with all 24 chapters complete

**Checkpoint**: Book content complete - all 24 chapters written and validated

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Final quality validation and deployment preparation

- [ ] T107 [P] Create book introduction page in docs/intro.md (book overview, how to navigate, prerequisites)
- [ ] T108 [P] Run book-wide link validation using markdown-link-check (zero broken internal/external links)
- [ ] T109 [P] Run Flesch-Kincaid analysis on all 24 chapters and generate readability report
- [ ] T110 [P] Validate all 27 functional requirements (FR-001 to FR-027) met
- [ ] T111 [P] Validate all 15 success criteria (SC-001 to SC-015) met
- [ ] T112 Create final accuracy validation report: cross-check all technical claims vs. primary sources
- [ ] T113 Create final accessibility validation report: all diagrams have alt text, all images have captions
- [ ] T114 Create final structural validation report: all chapters follow template, all cross-references accurate
- [ ] T115 [P] Create README.md at repository root with book description and deployment link placeholder
- [ ] T116 [P] Create LICENSE file at repository root (choose appropriate license for educational content)
- [ ] T117 [P] Create CONTRIBUTING.md at repository root (if open to contributions)
- [ ] T118 Configure GitHub Actions workflow in .github/workflows/deploy.yml for automatic GitHub Pages deployment
- [ ] T119 Run final local build `npm run build` (must complete with zero errors, zero warnings)
- [ ] T120 Run Lighthouse audit on local build (Performance ≥90, Accessibility ≥90)
- [ ] T121 Test responsive design on desktop (1920x1080), tablet (768x1024), mobile (375x667)
- [ ] T122 Deploy to GitHub Pages and verify site loads correctly at public URL
- [ ] T123 Test deployed site on Chrome, Firefox, Safari, Edge browsers
- [ ] T124 Update README.md with actual deployed site URL
- [ ] T125 Create Git release tag v1.0.0
- [ ] T126 Final review: navigate through entire deployed site verifying correctness and quality

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all content writing
- **User Stories 1-7 (Phases 3-9)**: All depend on Foundational phase completion
  - User stories CAN proceed in parallel (if staffed) since each module is independent
  - Or sequentially in priority order (P1 → P2 → P3 → P4 → P5 → P6 → P7)
- **Polish (Phase 10)**: Depends on all user stories (Phases 3-9) being complete

### User Story Dependencies

- **US1 (Part 1)**: Can start after Foundational - No dependencies on other stories
- **US2 (Module 1 - ROS 2)**: Can start after Foundational - Conceptually builds on US1 but independently writable
- **US3 (Module 2 - Digital Twin)**: Can start after Foundational - References US2 (URDF) but independently writable
- **US4 (Module 3 - Isaac)**: Can start after Foundational - References US2 & US3 but independently writable
- **US5 (Module 4 - VLA)**: Can start after Foundational - Synthesizes US2-US4 but independently writable
- **US6 (Capstone)**: Should start after US1-US5 complete (integrates all modules)
- **US7 (Future)**: Can start any time after Foundational - Independent content

### Within Each User Story

- Chapter writing tasks marked [P] can proceed in parallel (different files)
- Diagram creation marked [P] can proceed in parallel (different diagrams)
- Quality checks (readability, accuracy, links) should happen after all chapters in story written
- Local build test should happen after all content and diagrams complete for that story

### Parallel Opportunities

**Phase 1 (Setup)**: All tasks T002-T007 can run in parallel

**Phase 2 (Foundational)**:
- Research tasks T010-T014 can run in parallel (different sources)
- ADR creation tasks T019-T022 can run in parallel (different decisions)

**Phase 3 (US1)**:
- Chapter writing T025-T027 can run in parallel (3 chapters)
- Diagram creation T028-T030 can run in parallel (3 diagrams)

**Phase 4 (US2)**:
- Chapter writing T036-T039 can run in parallel (4 chapters)
- Diagram creation T040-T043 can run in parallel (4 diagrams)

**Phase 5 (US3)**:
- Chapter writing T049-T052 can run in parallel (4 chapters)
- Diagram creation T053-T056 can run in parallel (4 diagrams)

**Phase 6 (US4)**:
- Chapter writing T062-T065 can run in parallel (4 chapters)
- Diagram creation T066-T069 can run in parallel (4 diagrams)

**Phase 7 (US5)**:
- Chapter writing T075-T077 can run in parallel (3 chapters, Ch 19 should be last)
- Diagram creation T079-T081 can run in parallel (3 diagrams)

**Phase 8 (US6)**:
- Diagram creation T092-T093 can run in parallel (2 diagrams)

**Phase 9 (US7)**:
- Chapter writing T099-T100 can run in parallel (2 chapters)
- Diagram creation T101-T102 can run in parallel (2 diagrams)

**Phase 10 (Polish)**:
- Tasks T107-T111 can run in parallel (different validation types)
- Tasks T115-T117 can run in parallel (different documentation files)

**Cross-Phase Parallel**: Once Foundational complete, US1-US7 can ALL start in parallel if sufficient team capacity (7 parallel workstreams)

---

## Parallel Example: Module 1 (User Story 2)

```bash
# Launch all Module 1 chapters together:
Task T036: "Write Chapter 4 in docs/part2-modules/module1-ros2/chapter04-intro-ros2.md"
Task T037: "Write Chapter 5 in docs/part2-modules/module1-ros2/chapter05-nodes-topics-services.md"
Task T038: "Write Chapter 6 in docs/part2-modules/module1-ros2/chapter06-python-agents-rclpy.md"
Task T039: "Write Chapter 7 in docs/part2-modules/module1-ros2/chapter07-urdf-modeling.md"

# Launch all Module 1 diagrams together:
Task T040: "Create Mermaid sequence diagram for Chapter 4"
Task T041: "Create Mermaid diagram for Chapter 5"
Task T042: "Create pseudocode for Chapter 6"
Task T043: "Create Mermaid diagram for Chapter 7"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Part 1 Foundations)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational (T009-T024) - CRITICAL foundation
3. Complete Phase 3: User Story 1 (T025-T035)
4. **STOP and VALIDATE**: Test Part 1 independently
   - Can readers understand Physical AI after reading Part 1?
   - Does Part 1 meet acceptance criteria?
5. Deploy Part 1 to staging/demo if ready

### Incremental Delivery (One Module at a Time)

1. Complete Setup + Foundational → Book structure ready
2. Add US1 (Part 1) → Test independently → Deploy/Demo (MVP!)
3. Add US2 (Module 1 - ROS 2) → Test independently → Deploy/Demo
4. Add US3 (Module 2 - Digital Twin) → Test independently → Deploy/Demo
5. Add US4 (Module 3 - Isaac) → Test independently → Deploy/Demo
6. Add US5 (Module 4 - VLA) → Test independently → Deploy/Demo
7. Add US6 (Capstone) → Test integration → Deploy/Demo
8. Add US7 (Future) → Complete book → Deploy final version
9. Each module adds value without breaking previous content

### Parallel Team Strategy

With multiple writers/contributors:

1. Team completes Setup + Foundational together (T001-T024)
2. Once Foundational is done:
   - Writer A: User Story 1 (Part 1)
   - Writer B: User Story 2 (Module 1 - ROS 2)
   - Writer C: User Story 3 (Module 2 - Digital Twin)
   - Writer D: User Story 4 (Module 3 - Isaac)
   - Writer E: User Story 5 (Module 4 - VLA)
3. Each writer completes their module independently and validates quality
4. Integrate modules sequentially or in parallel
5. Complete US6 (Capstone) after all modules ready
6. Complete US7 (Future) any time
7. Final Polish phase done collaboratively

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [Story] label (US1-US7) maps task to specific user story for traceability
- Each user story delivers an independently valuable increment of the book
- Flesch-Kincaid grade level 10-12 MUST be validated for every chapter
- All technical claims MUST be validated against primary sources (sources.md)
- All diagrams MUST have alt text and figure captions (WCAG 2.1 Level AA)
- Docusaurus build MUST succeed with zero errors/warnings before deployment
- Commit after each completed chapter or logical group of tasks
- Stop at any checkpoint to validate story independently before proceeding
- MVP = Part 1 only (Chapters 1-3) - can deploy and get reader feedback early
- Avoid: vague tasks, same file conflicts, writing chapters out of dependency order
