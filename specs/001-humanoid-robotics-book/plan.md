# Implementation Plan: Humanoid Robotics & Physical AI Book

**Branch**: `001-humanoid-robotics-book` | **Date**: 2025-12-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-humanoid-robotics-book/spec.md`

## Summary

This plan outlines the strategy for creating "Humanoid Robotics & Physical AI — A Beginner's Guide to Embodied Intelligence," a comprehensive educational resource covering 24 chapters across 4 parts. The book teaches Physical AI, ROS 2, simulation (Gazebo/Unity), NVIDIA Isaac, and Vision-Language-Action pipelines through progressive, module-based learning. Content will be written using Spec-Kit Plus methodology, deployed as a Docusaurus static site to GitHub Pages, and validated against constitutional principles for accuracy, clarity, and reproducibility.

## Technical Context

**Language/Version**: Markdown/MDX, Node.js 20.x LTS (for Docusaurus build tooling)
**Primary Dependencies**: Docusaurus 3.x (latest stable), React (Docusaurus dependency), Mermaid (for diagrams as code)
**Storage**: Git repository (GitHub), static files in `/static/img/`, Markdown content in `/docs/`
**Testing**: Markdown link validation, Flesch-Kincaid readability analysis, Docusaurus build validation, Lighthouse accessibility/performance testing
**Target Platform**: Web (GitHub Pages deployment), responsive design for desktop/tablet/mobile
**Project Type**: Static site (Docusaurus-based educational content)
**Performance Goals**: Lighthouse Performance score ≥90, Lighthouse Accessibility score ≥90, page load <3s on 3G
**Constraints**: Flesch-Kincaid grade level 10-12, WCAG 2.1 Level AA accessibility, zero broken links, zero build warnings
**Scale/Scope**: 24 chapters, 4 major parts, 4 technical modules, estimated 60,000-100,000 words total content

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Content Accuracy and Verification ✅
- **Requirement**: All factual claims, code examples, and technical explanations must be verifiable and accurate
- **Plan Alignment**: Phase 0 includes research validation workflow; Phase 3 includes accuracy validation against primary sources
- **Validation Method**: Cross-reference claims with official documentation (ROS 2, NVIDIA Isaac, Whisper, etc.)

### II. Clarity for Target Audience ✅
- **Requirement**: Content for developers/technical writers with basic web dev knowledge; Flesch-Kincaid grade 10-12
- **Plan Alignment**: Section structure defines beginner-friendly chapter template; Phase 3 includes readability checks
- **Validation Method**: Flesch-Kincaid analysis tools, peer review for clarity

### III. Reproducibility (NON-NEGOTIABLE) ✅
- **Requirement**: Every tutorial, example, and workflow must be reproducible
- **Plan Alignment**: Chapter structure includes prerequisites, expected outputs, troubleshooting; code examples tested
- **Validation Method**: Independent tester follows chapter examples, documents blockers

### IV. Spec-Driven Development Integration ✅
- **Requirement**: Content creation follows Spec-Kit Plus methodology (spec → plan → tasks → implementation)
- **Plan Alignment**: This plan follows /sp.plan workflow; tasks.md will be generated via /sp.tasks; PHRs created for each session
- **Validation Method**: All artifacts present (spec.md, plan.md, tasks.md, PHRs, ADRs for significant decisions)

### V. Version Control and Collaboration ✅
- **Requirement**: Git for versioning, atomic commits, branch naming conventions, PR workflow
- **Plan Alignment**: Branch `001-humanoid-robotics-book` created; phased work breakdown enables incremental commits
- **Validation Method**: Git history shows atomic commits with clear messages; Docusaurus builds before merge

### VI. Deployment and Accessibility ✅
- **Requirement**: Continuous deployment to GitHub Pages, WCAG 2.1 Level AA, Lighthouse ≥90
- **Plan Alignment**: Phase 4 includes Docusaurus deployment configuration, GitHub Actions workflow, accessibility validation
- **Validation Method**: Lighthouse audit, automated accessibility testing, successful GitHub Pages deployment

**Gate Status**: ✅ PASSED - All constitutional principles addressed in plan

## Architecture Sketch

### 1. Overall Structural Layout

```
BOOK ARCHITECTURE

┌────────────────────────────────────────────────────────────┐
│ PART 1: Foundations of Physical AI & Humanoid Robotics    │
│ Chapters 1-3: Conceptual grounding                        │
│ Purpose: Build mental models before technical depth       │
└────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────┐
│ PART 2: Module-Based Learning (Core Technical Skills)     │
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐    │
│  │  MODULE 1    │  │  MODULE 2    │  │  MODULE 3    │    │
│  │  ROS 2       │  │  Digital     │  │  NVIDIA      │    │
│  │  (Ch 4-7)    │  │  Twin        │  │  Isaac       │    │
│  │              │  │  (Ch 8-11)   │  │  (Ch 12-15)  │    │
│  └──────────────┘  └──────────────┘  └──────────────┘    │
│                                                            │
│  ┌──────────────┐                                         │
│  │  MODULE 4    │                                         │
│  │  VLA         │                                         │
│  │  (Ch 16-19)  │                                         │
│  └──────────────┘                                         │
└────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────┐
│ PART 3: Capstone - Building the Autonomous Humanoid       │
│ Chapters 20-22: Integration of all modules                │
│ Purpose: Demonstrate complete system architecture         │
└────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────┐
│ PART 4: Future of Humanoid Robotics                       │
│ Chapters 23-24: Trends and career guidance                │
│ Purpose: Provide next steps and professional context      │
└────────────────────────────────────────────────────────────┘
```

### 2. Major Sections and Relationships

**Part 1 (Foundation)** → Establishes vocabulary and context
- Chapter 1: Defines Physical AI
- Chapter 2: Defines humanoid robots
- Chapter 3: Motivates embodied intelligence

**Part 2 (Modules)** → Each module builds on previous
- Module 1 (ROS 2): Communication foundation for all subsequent modules
- Module 2 (Digital Twin): Requires ROS 2 knowledge for simulation
- Module 3 (Isaac): Requires simulation knowledge + ROS 2 integration
- Module 4 (VLA): Synthesizes all prior modules (ROS for action, simulation for testing, Isaac for perception)

**Part 3 (Capstone)** → Integrates all modules into one system
- Chapter 20: Shows architectural diagram connecting all components
- Chapter 21: Demonstrates integration patterns
- Chapter 22: Walkthrough of complete autonomous humanoid

**Part 4 (Future)** → Provides context and next steps
- Chapter 23: Emerging trends (next decade)
- Chapter 24: Career paths and learning roadmap

### 3. Flow of Ideas Across Modules

```
Foundation (Ch 1-3)
    ↓
    Provides: Vocabulary, motivation, mental models
    ↓
Module 1: ROS 2 (Ch 4-7)
    ↓
    Provides: Communication primitives (nodes, topics, actions), URDF modeling
    ↓
Module 2: Digital Twin (Ch 8-11)
    ↓
    Requires: URDF from Module 1
    Provides: Physics simulation, sensor simulation, visualization
    ↓
Module 3: Isaac (Ch 12-15)
    ↓
    Requires: ROS 2 communication (Module 1), Simulation environments (Module 2)
    Provides: Perception (VSLAM, object detection), Navigation (Nav2), Learning (RL)
    ↓
Module 4: VLA (Ch 16-19)
    ↓
    Requires: ROS 2 actions (Module 1), Isaac perception (Module 3)
    Provides: Multimodal intelligence (speech → LLM → action planning)
    ↓
Capstone (Ch 20-22)
    ↓
    Synthesizes: All modules into complete autonomous humanoid system
    ↓
Future (Ch 23-24)
    ↓
    Provides: Professional context and learning trajectory
```

### 4. How Modules Connect to Capstone Humanoid Project

The capstone (Chapters 20-22) demonstrates a complete autonomous humanoid that:

1. **Uses ROS 2 (Module 1)** for:
   - Inter-component communication (perception → planning → control)
   - Action execution (navigation, manipulation)
   - URDF-based robot model

2. **Uses Digital Twin (Module 2)** for:
   - Safe testing environment before real-world deployment
   - Physics-accurate simulation of walking, grasping, balancing
   - Sensor data generation for perception testing

3. **Uses Isaac (Module 3)** for:
   - Visual SLAM to map environment
   - Object detection to identify targets
   - Path planning for navigation
   - RL-trained skills (walking, grasping)

4. **Uses VLA (Module 4)** for:
   - Voice command input ("bring me the cup")
   - LLM-based task planning (decompose into subtasks)
   - Vision-language grounding (identify "the cup" in scene)

**Integration Example** (Chapter 19 capstone implementation):
```
User says: "Pick up the red cup from the table"
    ↓ Whisper (Module 4)
Text: "Pick up the red cup from the table"
    ↓ LLM Planning (Module 4)
Task Plan: [navigate to table, identify red cup, grasp cup, return]
    ↓ ROS 2 Actions (Module 1)
Navigation action sent to Nav2 (Module 3)
    ↓ Isaac Perception (Module 3)
VSLAM builds map, object detection finds "red cup"
    ↓ Manipulation Action (Module 1 + Isaac RL)
Grasp action executed in simulation (Module 2)
    ↓ Result
Autonomous humanoid completes task
```

## Section Structure (Standard Chapter Template)

All 24 chapters will follow this consistent structure for clarity and beginner-friendliness:

### Chapter Template

```markdown
# Chapter X — [Chapter Title]

## Learning Objectives

By the end of this chapter, you will be able to:
- [Specific measurable objective 1]
- [Specific measurable objective 2]
- [Specific measurable objective 3]

## Prerequisites

**Required Knowledge**:
- [Previous chapter or external knowledge needed]

**Tools/Environment** (if applicable):
- [Software versions, installation links]

---

## Introduction (Why This Matters)

[1-2 paragraphs explaining the relevance of this topic to humanoid robotics]
[Real-world motivation or example]

---

## Concept 1: [Core Concept Name]

### What It Is

[Clear definition in plain language]

### Why It Matters for Humanoid Robots

[Specific application to embodied intelligence]

### Visual Explanation

[Mermaid diagram or illustration]
**Figure X.1**: [Caption with alt text]

### Key Terminology

- **Term 1**: Definition
- **Term 2**: Definition

---

## Concept 2: [Core Concept Name]

[Same structure as Concept 1]

---

## How [Concepts] Work Together

[Integration section showing how concepts in this chapter connect]

### Example Scenario

**Scenario**: [Concrete example, e.g., "A humanoid robot navigating a room"]

**Step-by-step flow**:
1. [Action using Concept 1]
2. [Action using Concept 2]
3. [Result]

### Conceptual Code Flow (if applicable)

```python
# High-level pseudocode showing concept application
# NOT production code, but conceptual understanding
```

**Explanation**: [Line-by-line walkthrough of key points]

---

## Common Questions & Misconceptions

**Q**: [Common question from beginners]
**A**: [Clear answer addressing the confusion]

**Q**: [Another common question]
**A**: [Clear answer]

---

## Connection to Other Modules

[How this chapter's concepts connect to other modules]
- **Links to Module 1 (ROS 2)**: [Specific connection]
- **Links to Module 3 (Isaac)**: [Specific connection]
- **Role in Capstone**: [How this appears in final integration]

---

## Summary

**In this chapter, you learned**:
- [Key takeaway 1]
- [Key takeaway 2]
- [Key takeaway 3]

**Skills Gained**:
- ✅ [Skill 1]
- ✅ [Skill 2]
- ✅ [Skill 3]

**Next Steps**:
- Chapter [X+1] will cover [preview of next topic]
- To go deeper, explore [external resource with link]

---

## References

[1] [Author/Organization]. [Resource Title]. [URL]. Accessed [Date].
[2] [Author/Organization]. [Resource Title]. [URL]. Accessed [Date].
```

### Section Arrangement Rationale

1. **Learning Objectives** — Sets expectations upfront (aligned with educational best practices)
2. **Prerequisites** — Ensures readers have necessary background (reproducibility)
3. **Introduction** — Motivates the topic (engagement and context)
4. **Concept Sections** — Progressive disclosure of 2-4 core concepts per chapter
5. **Integration Section** — Shows how concepts work together (synthesis)
6. **Q&A** — Addresses common confusions proactively (clarity)
7. **Connections** — Links to other modules (big picture understanding)
8. **Summary** — Reinforces learning (retention)
9. **References** — Enables fact-checking and deeper exploration (accuracy, reproducibility)

### Visual/Diagram Placement

- **Diagrams as Code**: Use Mermaid for architecture diagrams, flowcharts, sequence diagrams
- **Placement**: Immediately after concept introduction, before detailed explanation
- **Accessibility**: All diagrams include alt text and figure captions
- **Types**:
  - Architecture diagrams (system components and connections)
  - Flowcharts (decision trees, workflows)
  - Sequence diagrams (ROS 2 message passing, VLA pipeline)
  - Concept maps (relationships between ideas)

### Tone & Accessibility Level

- **Tone**: Professional yet approachable; enthusiastic about robotics without hyperbole
- **Voice**: Second person ("you will learn"), active voice preferred
- **Technical depth**: Conceptual understanding prioritized over implementation details
- **Analogies**: Use when helpful for complex concepts (e.g., "ROS 2 nodes are like microservices")
- **Jargon policy**: Define specialized terms on first use; maintain glossary
- **Reading level**: Target Flesch-Kincaid grade 10-12 (high school to college sophomore)

## Research Approach

### Research-Concurrent Workflow

**Philosophy**: Research happens alongside writing, not entirely upfront. This allows:
- Discovery of knowledge gaps during content creation
- Just-in-time research for specific chapter needs
- Iterative refinement as understanding deepens

### Research Phases

#### Phase 0: Foundation Research (Before Writing Begins)

**Scope**: High-level validation of book structure and module feasibility

**Activities**:
1. Verify ROS 2, Gazebo, Unity, Isaac, Whisper official documentation exists and is accessible
2. Confirm module prerequisites (e.g., does Isaac integrate with ROS 2 as claimed?)
3. Identify major version considerations (ROS 2 Humble vs. Iron, Isaac Sim 2023 vs. 2024)
4. Validate VLA pipeline feasibility (Whisper → LLM → ROS 2 action flow)

**Deliverable**: `research.md` with:
- Confirmed technology versions
- Links to primary sources (official docs)
- Known limitations or deprecations
- Research tasks for each module

#### Concurrent Research (During Chapter Writing)

**Trigger**: Whenever writing a chapter and encountering:
- Uncertain technical claim
- Need for specific example or code snippet
- Ambiguous terminology
- Conflicting information across sources

**Process**:
1. **Document Question**: Add to chapter's research notes (inline comment or separate file)
2. **Consult Primary Sources**: Official documentation first (ROS 2 wiki, NVIDIA Isaac docs, etc.)
3. **Validate Claim**: Cross-reference with 2+ authoritative sources when possible
4. **Record Finding**: Update research.md with source, date accessed, relevant quote/link
5. **Integrate into Chapter**: Write content based on verified information

### Methods for Validating Technical Claims

1. **Official Documentation Priority**:
   - ROS 2: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
   - NVIDIA Isaac: [https://docs.omniverse.nvidia.com/isaacsim/latest/](https://docs.omniverse.nvidia.com/isaacsim/latest/)
   - Gazebo: [https://gazebosim.org/docs](https://gazebosim.org/docs)
   - Whisper: [https://github.com/openai/whisper](https://github.com/openai/whisper)

2. **Peer-Reviewed Sources** (when available):
   - Academic papers on SLAM, reinforcement learning, VLA architectures
   - IEEE/ACM publications on robotics

3. **Reputable Industry Sources**:
   - Official blog posts from ROS, NVIDIA, Unity
   - Conference talks (ROSCon, NVIDIA GTC)

4. **Testing & Experimentation**:
   - For code examples: Run and verify output matches description
   - For conceptual flows: Validate against documentation or expert review

### Strategies for Identifying Credible Sources

**Tier 1 (Highest Credibility)**:
- Official project documentation (ros.org, nvidia.com/isaac)
- Peer-reviewed academic papers (arXiv, IEEE, ACM)
- Official GitHub repositories with active maintenance

**Tier 2 (High Credibility)**:
- Technical blog posts from project maintainers
- Conference presentations from core developers
- Books from recognized publishers (O'Reilly, Packt) — use as supplementary

**Tier 3 (Use with Caution)**:
- Community tutorials (verify claims independently)
- Stack Overflow answers (check vote count, date, and against official docs)
- Medium articles (only from verified experts)

**Red Flags (Avoid)**:
- Outdated content (>2 years old for rapidly evolving tools like ROS 2, Isaac)
- Sources without attribution or references
- Content with obvious errors or inconsistencies

### Ensuring Alignment with Robotics Documentation

**Alignment Checks**:
1. **Terminology Consistency**: Use exact terms from official docs (e.g., "action client" not "action requester")
2. **Version Specificity**: Specify ROS 2 distribution (Humble, Iron), Isaac version (2023.1.1)
3. **Code Example Validation**: Test code snippets against documented APIs
4. **Conceptual Accuracy**: Verify architectural diagrams match official system designs

**Documentation Tracking**:
- Maintain a `sources.md` file listing all primary sources with access dates
- Link each chapter to relevant documentation sections
- Note version-specific information clearly

### Integration of Findings into Chapters Efficiently

**Workflow**:
1. **Research Notes**: Keep per-chapter research notes during writing
2. **Batch Updates**: After researching 3-5 questions, update chapter content in one session
3. **Citation as You Write**: Add references inline (APA style) immediately when using a source
4. **Visual Capture**: Screenshot or diagram official architecture diagrams for reference (create original diagrams inspired by, not copied)

**Efficiency Practices**:
- Use browser bookmarks or reference manager for frequently accessed docs
- Create a "snippets" file for commonly referenced code patterns
- Maintain a glossary document for consistent terminology across chapters

## Decisions Requiring Documentation

### Decision 1: Depth of Technical Detail

**Available Options**:

| Option | Description | Target Audience |
|--------|-------------|-----------------|
| A | Conceptual overview only (no code, high-level diagrams) | Complete beginners, non-programmers |
| B | Conceptual + pseudocode flows (explanatory code, not production) | Developers new to robotics |
| C | Conceptual + working code examples (executable, production-ready) | Experienced developers seeking hands-on |

**Pros and Cons**:

- **Option A**:
  - ✅ Accessible to widest audience
  - ✅ Fastest to produce content
  - ❌ May frustrate developers who want to try examples
  - ❌ Harder to validate accuracy without executable code

- **Option B**:
  - ✅ Balances accessibility and technical depth
  - ✅ Conceptual code enforces understanding over copy-paste
  - ✅ Easier to maintain (less fragile than full code)
  - ❌ Readers cannot directly execute examples
  - ✅ Aligns with "conceptual understanding" goal

- **Option C**:
  - ✅ Maximum hands-on learning
  - ✅ Validates accuracy through execution
  - ❌ Requires full environment setup (ROS 2, Isaac installation)
  - ❌ Code becomes outdated faster
  - ❌ May overwhelm beginners with setup complexity

**Tradeoffs**:
- **Accessibility vs. Depth**: Option A maximizes accessibility but sacrifices actionable learning
- **Maintenance vs. Utility**: Option C provides most utility but highest maintenance burden
- **Learning Style**: Option B supports "learn by understanding" vs. Option C "learn by doing"

**Final Choice**: **Option B — Conceptual + Pseudocode Flows**

**Rationale**:
1. **Aligns with Spec FR-020**: "Book MUST use diagrams, code examples, and visual aids to illustrate complex concepts" — pseudocode satisfies "code examples" while prioritizing concepts
2. **Aligns with Constitution Principle II (Clarity)**: Target audience has basic programming knowledge, can read pseudocode
3. **Aligns with Constitution Principle III (Reproducibility)**: Conceptual flows are "reproducible" as mental models, even if not executable
4. **Reduces Maintenance Burden**: Pseudocode less brittle than full dependency chains
5. **Accessibility**: Readers without ROS 2/Isaac installed can still follow along and learn concepts
6. **Optional Deep Dive**: Can provide links to external repos with full working code for readers who want hands-on practice

**Implementation**:
- Use Python-like pseudocode with clear comments
- Annotate with "Conceptual Flow" labels
- Provide "For hands-on implementation, see [external repo link]" where applicable
- For critical examples (e.g., ROS 2 publisher/subscriber), provide minimal working code in appendix or companion repo

---

### Decision 2: Use of Conceptual vs. Practical Examples

**Available Options**:

| Option | Description | Example Type |
|--------|-------------|--------------|
| A | Toy examples (simplified, abstract) | "Robot moves from point A to B" |
| B | Realistic scenarios (complex, domain-specific) | "Humanoid navigates hospital hallway to deliver medicine" |
| C | Hybrid (start simple, build to realistic) | "Start with 'move forward', build to 'navigate hallway'" |

**Pros and Cons**:

- **Option A (Toy Examples)**:
  - ✅ Easy to understand
  - ✅ Isolates concepts clearly
  - ❌ May feel artificial or disconnected from real robotics
  - ❌ Harder to motivate "why this matters"

- **Option B (Realistic Scenarios)**:
  - ✅ High engagement and motivation
  - ✅ Shows real-world applicability
  - ❌ Complexity may obscure core concepts
  - ❌ Requires more context/setup

- **Option C (Hybrid)**:
  - ✅ Best of both worlds
  - ✅ Progressive complexity matches learning curve
  - ✅ Early chapters use simple examples, later chapters use realistic scenarios
  - ❌ Requires careful scaffolding

**Tradeoffs**:
- **Clarity vs. Engagement**: Toy examples clearer but less engaging; realistic scenarios engaging but potentially confusing
- **Concept Isolation vs. Integration**: Toy examples isolate concepts; realistic scenarios show integration (but may conflate multiple concepts)

**Final Choice**: **Option C — Hybrid (Progressive Complexity)**

**Rationale**:
1. **Aligns with Spec User Stories**: P1 (Foundations) uses simple examples; P5 (VLA) and P6 (Capstone) use realistic scenarios
2. **Aligns with Constitution Principle II**: "Progressive disclosure: simple concepts first, advanced topics later"
3. **Pedagogical Best Practice**: Scaffolding from simple to complex improves retention
4. **Motivation**: Realistic scenarios in capstone chapters provide "payoff" for earlier conceptual learning

**Implementation**:
- **Part 1 (Foundations)**: Toy examples ("What if a robot needs to sense its environment?")
- **Part 2 (Modules)**: Graduated complexity
  - Early chapters in each module: Simplified examples
  - Later chapters: Increasingly realistic scenarios
- **Part 3 (Capstone)**: Full realistic scenario (autonomous humanoid in home environment)

---

### Decision 3: Diagram Types and Formats

**Available Options**:

| Option | Format | Creation Method | Example |
|--------|--------|-----------------|---------|
| A | Static images (PNG/SVG) | Graphic design tools (Figma, Illustrator) | Hand-crafted diagrams |
| B | Diagrams as code (Mermaid) | Markdown code blocks | Flowcharts, sequence diagrams |
| C | AI-generated diagrams | Claude, DALL-E with prompts | Conceptual illustrations |
| D | Hybrid (Mermaid + static for complex cases) | Mix of B and A | Simple diagrams in Mermaid, complex in SVG |

**Pros and Cons**:

- **Option A (Static Images)**:
  - ✅ Full design control
  - ✅ Can create highly polished visuals
  - ❌ Not version-controllable as code
  - ❌ Harder to update/maintain
  - ❌ Requires graphic design skills/tools

- **Option B (Diagrams as Code — Mermaid)**:
  - ✅ Version-controlled alongside content
  - ✅ Easy to update
  - ✅ Renders in Docusaurus natively
  - ✅ Accessible (text-based, screen reader friendly)
  - ❌ Limited to certain diagram types (flowcharts, sequence, class diagrams)
  - ❌ Less design flexibility

- **Option C (AI-Generated)**:
  - ✅ Fast to produce
  - ✅ Can generate novel conceptual illustrations
  - ❌ Quality/accuracy varies
  - ❌ May require significant prompt engineering
  - ❌ Licensing/attribution concerns

- **Option D (Hybrid)**:
  - ✅ Flexibility for different diagram needs
  - ✅ Mermaid for technical diagrams, static for illustrations
  - ❌ Inconsistent tooling/workflow

**Tradeoffs**:
- **Maintainability vs. Polish**: Code-based diagrams easier to maintain but less polished than hand-crafted
- **Accessibility vs. Complexity**: Mermaid diagrams accessible but limited in complexity
- **Speed vs. Quality**: AI-generated fast but variable quality

**Final Choice**: **Option D — Hybrid (Mermaid Primary, SVG for Complex Cases)**

**Rationale**:
1. **Aligns with Constitution Technical Standards**: "Prefer Mermaid diagrams (as code) over static images when feasible"
2. **Aligns with Spec FR-020**: "Book MUST use diagrams, code examples, and visual aids to illustrate complex concepts"
3. **Version Control**: Mermaid diagrams live in Markdown, fully version-controlled
4. **Accessibility**: Mermaid generates accessible SVG; can add alt text in Markdown
5. **Pragmatism**: Complex system architectures (e.g., full VLA pipeline) may exceed Mermaid capabilities, require hand-crafted SVG

**Implementation**:
- **Default**: Use Mermaid for all diagrams
  - Flowcharts (chapter conceptual flows)
  - Sequence diagrams (ROS 2 message passing)
  - Architecture diagrams (system components)
- **Exception**: Use SVG (created in Figma/Excalidraw) for:
  - Complex multi-layer architectures (e.g., Chapter 20 complete system)
  - Illustrations requiring custom visuals (robot diagrams, sensor representations)
- **AI-Generated**: Use only for conceptual inspiration, not final diagrams (quality control)
- **Accessibility**: All diagrams include alt text and figure captions

---

### Decision 4: Tone and Accessibility Level

**Available Options**:

| Option | Tone | Formality | Example Phrase |
|--------|------|-----------|----------------|
| A | Academic/Formal | High | "The robotic system employs a distributed architecture..." |
| B | Conversational/Casual | Low | "So, robots basically talk to each other like this..." |
| C | Professional/Approachable | Medium | "ROS 2 enables robots to communicate through a publish-subscribe pattern." |

**Pros and Cons**:

- **Option A (Academic/Formal)**:
  - ✅ Authoritative, credible
  - ✅ Aligns with scientific/technical writing standards
  - ❌ May intimidate beginners
  - ❌ Can feel dry or impersonal
  - ❌ Higher Flesch-Kincaid grade level (13+)

- **Option B (Conversational/Casual)**:
  - ✅ Highly accessible, engaging
  - ✅ Reduces intimidation factor
  - ❌ May undermine credibility
  - ❌ Can feel unprofessional
  - ❌ Risk of being too simplistic or condescending

- **Option C (Professional/Approachable)**:
  - ✅ Balances authority and accessibility
  - ✅ Maintains technical credibility while staying readable
  - ✅ Flesch-Kincaid grade 10-12 achievable
  - ❌ Requires careful word choice to balance

**Tradeoffs**:
- **Credibility vs. Accessibility**: Formal tone more credible but less accessible; casual tone more accessible but less credible
- **Engagement vs. Professionalism**: Conversational tone engages but may seem unserious; formal tone professional but may disengage

**Final Choice**: **Option C — Professional/Approachable**

**Rationale**:
1. **Aligns with Constitution Principle II**: "Clarity for Target Audience — developers and technical writers" suggests professional tone
2. **Aligns with Constitution**: "Writing clarity target: Flesch-Kincaid grade level 10-12" — achievable with professional/approachable tone
3. **Credibility**: Technical book requires authority; overly casual undermines trust
4. **Accessibility**: "Approachable" modifier ensures beginners aren't intimidated
5. **Industry Standard**: Most successful technical books (O'Reilly, Packt) use professional/approachable tone

**Implementation**:
- **Voice**: Second person ("you will learn"), active voice
- **Sentence Structure**: Vary length; avoid overly complex nested clauses
- **Jargon**: Define on first use, use consistently thereafter
- **Examples**: Use relatable scenarios ("imagine a robot in a hospital" not "consider an autonomous agent in a clinical environment")
- **Enthusiasm**: Show passion for robotics without hyperbole ("exciting developments" not "mind-blowing revolutionary paradigm shifts")
- **Inclusivity**: Gender-neutral language, avoid assumptions about reader background beyond stated prerequisites

---

## Testing & Validation Strategy

### 1. Accuracy Validation Against Primary Sources

**Process**:
1. **Pre-Writing Check** (Phase 0 Research):
   - Verify all module technologies exist and are documented
   - Confirm major architectural claims (e.g., "Isaac integrates with ROS 2")

2. **During Writing**:
   - Every technical claim must link to primary source (ROS 2 docs, Isaac docs, etc.)
   - Code examples must reference API documentation
   - Conceptual diagrams must align with official architecture diagrams

3. **Post-Writing Review**:
   - Cross-reference each chapter against primary sources
   - Validate version-specific information (ROS 2 Humble, Isaac 2023.1.1)
   - Check for outdated information (tools evolve rapidly)

**Validation Checklist (Per Chapter)**:
- [ ] All factual claims have primary source citations
- [ ] All code examples reference official API docs
- [ ] All terminology matches official documentation
- [ ] Version-specific information is accurate and clearly labeled
- [ ] No contradictions with official documentation

**Tools**:
- Reference management (Zotero or similar) for tracking sources
- Automated link checker for validating external links
- Manual review against primary docs

---

### 2. Structural Validation (Logical Flow, Consistency)

**Process**:
1. **Chapter-Level Validation**:
   - Each chapter follows standard template structure
   - Learning objectives align with content and summary
   - Prerequisites are accurate and sufficient
   - Concepts flow logically (simple → complex)

2. **Module-Level Validation**:
   - Chapters within a module build progressively
   - Skills from earlier chapters prerequisites for later chapters
   - Module learning objectives met by end of final chapter

3. **Book-Level Validation**:
   - Part 1 provides sufficient foundation for Part 2
   - Modules reference each other correctly (Module 3 correctly references Module 1 concepts)
   - Capstone (Part 3) integrates all modules coherently
   - No orphaned concepts (every concept used in capstone or later chapters)

**Validation Checklist (Per Chapter)**:
- [ ] Chapter follows standard template structure
- [ ] Learning objectives match content and summary
- [ ] Prerequisites are complete and accurate
- [ ] Concepts ordered logically (foundational → advanced)
- [ ] Transitions between sections are smooth
- [ ] Cross-references to other chapters are accurate

**Tools**:
- Outline review (check against book architecture diagram)
- Dependency mapping (visual graph of chapter prerequisites)

---

### 3. Beginner Accessibility Checks

**Process**:
1. **Readability Analysis**:
   - Run Flesch-Kincaid readability test on each chapter
   - Target: Grade level 10-12
   - Flag chapters above grade 13 for simplification

2. **Jargon Audit**:
   - Identify all technical terms in chapter
   - Verify each is defined on first use
   - Check glossary for consistency

3. **Assumption Validation**:
   - Review prerequisites section
   - Ensure no implicit assumptions beyond stated prerequisites
   - Flag concepts that assume knowledge not covered in earlier chapters

4. **Peer Review (if possible)**:
   - Recruit beta readers from target audience (developers new to robotics)
   - Gather feedback on clarity, pacing, difficulty
   - Iterate on confusing sections

**Validation Checklist (Per Chapter)**:
- [ ] Flesch-Kincaid grade level 10-12
- [ ] All technical terms defined on first use
- [ ] No implicit assumptions beyond stated prerequisites
- [ ] Code examples include sufficient explanatory comments
- [ ] Analogies used appropriately (not overly simplified or confusing)

**Tools**:
- Hemingway Editor or similar readability analyzer
- Glossary document (track all defined terms)
- Beta reader feedback form

---

### 4. Diagram Clarity and Correctness

**Process**:
1. **Visual Clarity**:
   - Diagrams are legible at typical screen sizes (1920x1080, 1366x768, mobile)
   - Text in diagrams is readable (minimum 12pt font equivalent)
   - Colors have sufficient contrast (WCAG AA minimum)
   - Arrows and flow direction are unambiguous

2. **Technical Accuracy**:
   - Diagram architecture matches official documentation
   - Labels match terminology from text
   - No contradictions with written content

3. **Accessibility**:
   - All diagrams have alt text describing content
   - Figure captions explain key points
   - Diagrams supplement (not replace) text explanations

**Validation Checklist (Per Diagram)**:
- [ ] Legible on desktop and mobile
- [ ] Sufficient color contrast (WCAG AA)
- [ ] Technically accurate (matches official docs)
- [ ] Labels match terminology in text
- [ ] Alt text describes diagram content
- [ ] Figure caption explains significance

**Tools**:
- Color contrast checker (WebAIM)
- Manual review on multiple device sizes
- Cross-reference with official documentation diagrams

---

### 5. Validation That All Modules Support Capstone

**Process**:
1. **Backward Tracing**:
   - Start with capstone chapters (20-22)
   - Identify every concept, tool, and technique used
   - Trace back to earlier chapters where these are taught
   - Ensure all capstone dependencies are covered

2. **Forward Validation**:
   - For each module (Modules 1-4), identify skills taught
   - Verify these skills are used in capstone
   - Flag any "orphaned" skills not used in capstone (consider removing or justifying)

3. **Integration Testing**:
   - Create a dependency matrix:
     - Rows: Capstone components (ROS 2 communication, Isaac perception, VLA pipeline)
     - Columns: Module chapters (Ch 4-19)
     - Cells: Mark where capstone component depends on module chapter
   - Ensure no empty rows (every capstone component has module support)

**Validation Checklist**:
- [ ] Every concept in capstone (Ch 20-22) is introduced in earlier chapters
- [ ] Every module contributes to at least one capstone component
- [ ] Capstone integration chapter (Ch 21) references all four modules explicitly
- [ ] No contradictions between module chapters and capstone implementation

**Tools**:
- Dependency matrix (spreadsheet or diagram)
- Manual tracing through content

---

### 6. Deployment Validation (Docusaurus Build + GitHub Pages Publishing)

**Process**:
1. **Local Build Validation**:
   - Run `npm run build` locally
   - Verify zero errors, zero warnings
   - Check generated site in `/build` directory
   - Test navigation, search, responsiveness

2. **Lighthouse Audit**:
   - Run Lighthouse on generated site
   - Verify Performance ≥90, Accessibility ≥90
   - Fix any failing audits

3. **Link Validation**:
   - Run automated link checker (e.g., `markdown-link-check`)
   - Verify zero broken internal links
   - Verify all external links resolve (primary sources)

4. **GitHub Pages Deployment**:
   - Configure GitHub Actions workflow for auto-deployment
   - Test deployment to GitHub Pages staging environment
   - Verify site loads correctly on public URL
   - Test on multiple browsers (Chrome, Firefox, Safari, Edge)

**Validation Checklist**:
- [ ] Docusaurus builds with zero errors, zero warnings
- [ ] Lighthouse Performance score ≥90
- [ ] Lighthouse Accessibility score ≥90
- [ ] Zero broken internal links
- [ ] All external links resolve
- [ ] Site deploys successfully to GitHub Pages
- [ ] Site loads correctly on Chrome, Firefox, Safari, Edge
- [ ] Responsive design works on mobile, tablet, desktop

**Tools**:
- Docusaurus CLI (`npm run build`, `npm run serve`)
- Lighthouse (browser DevTools)
- `markdown-link-check` or similar link validator
- GitHub Actions for CI/CD

---

## Technical Requirements

### 1. APA Citation Style

**Requirement**: All references must follow APA 7th edition style as defined in Constitution (Content Standards → Citation and Attribution).

**Implementation**:
- **In-Text Citations**: Use author-date format: (Author, Year) or Author (Year)
- **Reference List**: Alphabetical by author, hanging indent, APA format
- **Online Resources**: Include URL and access date
- **Software/Tools**: Cite official documentation or GitHub repository

**Example**:
```markdown
ROS 2 uses a publish-subscribe pattern for inter-node communication (Open Robotics, 2023).

## References

Open Robotics. (2023). *ROS 2 Documentation: Concepts*.
    https://docs.ros.org/en/humble/Concepts.html. Accessed December 5, 2025.

NVIDIA Corporation. (2023). *Isaac Sim Documentation*.
    https://docs.omniverse.nvidia.com/isaacsim/latest/. Accessed December 5, 2025.
```

**Tooling**:
- Maintain `sources.md` file with all references in APA format
- Copy references to end of each chapter as needed

---

### 2. AI-Generated or Original Diagrams Only

**Requirement**: No use of copyrighted diagrams from external sources without permission.

**Implementation**:
- **Mermaid Diagrams**: Original, created as code (preferred)
- **SVG Diagrams**: Original, created in Figma/Excalidraw
- **AI-Generated**: Only for conceptual inspiration; all final diagrams original
- **Inspired by Official Docs**: Allowed, but must be redrawn, not copied

**Citation for Inspired Diagrams**:
```markdown
**Figure 4.2**: ROS 2 architecture (inspired by Open Robotics, 2023)
```

---

### 3. Clear, Simple Writing Focused on Conceptual Understanding

**Requirement**: Prioritize conceptual understanding over implementation details.

**Implementation**:
- **Pseudocode > Production Code**: Use conceptual flows instead of full implementations
- **Diagrams > Prose**: Use visuals to explain complex systems
- **Analogies**: Employ when helpful (e.g., "ROS 2 topics are like radio channels")
- **No Deep Math**: Avoid equations unless absolutely necessary; explain conceptually instead
- **Glossary**: Maintain term definitions for consistency

**Example**:
```markdown
BAD: "The SLAM algorithm minimizes the objective function J(x) = Σᵢ ||zᵢ - h(xᵢ)||²"

GOOD: "Visual SLAM builds a map by combining what the robot sees (camera images)
with where it thinks it is (estimated position). As the robot moves, it refines
both the map and its position to reduce errors."
```

---

### 4. No Deep Mathematics or Complex Code Walkthroughs

**Requirement**: Avoid intimidating beginners with heavy math or line-by-line code analysis.

**Implementation**:
- **Math Policy**:
  - Explain concepts without equations
  - If equation necessary, provide intuitive explanation alongside
  - No derivations or proofs
- **Code Policy**:
  - Conceptual pseudocode, not production code
  - High-level flow diagrams instead of line-by-line walkthroughs
  - Link to external repos for readers who want full implementations

**Example**:
```markdown
BAD:
```python
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    # ... 10 more lines
```

GOOD:
```markdown
ROS 2 represents robot orientation using quaternions (4 numbers: x, y, z, w).
To convert to human-readable roll/pitch/yaw angles, use built-in transformation
functions. This avoids gimbal lock issues common with Euler angles.
```
```

---

### 5. Alignment with Constitutional Principles and Specification Requirements

**Cross-Check Matrix**:

| Constitution Principle | Plan Alignment | Validation Method |
|------------------------|----------------|-------------------|
| I. Content Accuracy and Verification | ✅ Research approach validates claims | Accuracy validation checklist |
| II. Clarity for Target Audience | ✅ Professional/approachable tone, FK 10-12 | Readability analysis |
| III. Reproducibility | ✅ Conceptual flows reproducible | Peer review, independent test |
| IV. Spec-Driven Development | ✅ This plan follows /sp.plan workflow | Artifacts present (plan.md, research.md) |
| V. Version Control and Collaboration | ✅ Phased work breakdown enables commits | Git history review |
| VI. Deployment and Accessibility | ✅ Deployment validation phase | Lighthouse audit, GitHub Pages test |

| Spec Requirement | Plan Alignment | Validation Method |
|------------------|----------------|-------------------|
| FR-001 to FR-023 (Content) | ✅ Chapter template, module structure | Structural validation |
| FR-024 (Docusaurus deployment) | ✅ Deployment validation phase | Build and deploy test |
| FR-025 (Markdown/MDX format) | ✅ Chapter template in Markdown | File format check |
| FR-026 (Table of contents) | ✅ Docusaurus auto-generates from structure | Navigation test |
| FR-027 (WCAG 2.1 Level AA) | ✅ Accessibility validation | Lighthouse audit |
| SC-001 to SC-015 (Success Criteria) | ✅ Testing & validation strategy | Per-criterion validation |

---

## Phased Work Breakdown

### Phase 0: Research

**Duration**: 2-3 weeks (concurrent with early writing)

**Objectives**:
- Explore foundational robotics concepts
- Gather authoritative sources
- Validate initial assumptions about module feasibility

**Activities**:

1. **Explore Foundational Robotics Concepts**:
   - Review ROS 2 documentation (architecture, communication patterns)
   - Review Gazebo/Unity simulation documentation
   - Review NVIDIA Isaac documentation (perception, navigation, learning)
   - Review VLA literature (academic papers, blog posts)

2. **Gather Authoritative Sources**:
   - Identify primary sources for each module:
     - Module 1: ROS 2 official docs, ROSCon talks
     - Module 2: Gazebo docs, Unity Robotics Hub
     - Module 3: NVIDIA Isaac docs, GTC presentations
     - Module 4: Whisper GitHub, LLM robotics papers
   - Create `sources.md` with APA citations

3. **Validate Initial Assumptions**:
   - Confirm ROS 2 + Isaac integration feasible
   - Confirm VLA pipeline (Whisper → LLM → ROS 2) architecturally sound
   - Identify any module dependencies or limitations
   - Document in `research.md`

**Deliverables**:
- ✅ `research.md` — Research findings and decisions
- ✅ `sources.md` — APA bibliography of primary sources
- ✅ Module feasibility validation (all modules confirmed feasible)

---

### Phase 1: Foundation

**Duration**: 2-3 weeks

**Objectives**:
- Establish book layout (Docusaurus configuration)
- Create chapter skeletons (outline for all 24 chapters)
- Finalize tone, style, and structure (chapter template)

**Activities**:

1. **Establish Book Layout**:
   - Initialize Docusaurus project
   - Configure `docusaurus.config.js` (title, theme, navigation)
   - Set up directory structure:
     ```
     docs/
     ├── part1-foundations/
     │   ├── chapter01-what-is-physical-ai.md
     │   ├── chapter02-understanding-humanoid-robots.md
     │   └── chapter03-why-embodied-intelligence-matters.md
     ├── part2-modules/
     │   ├── module1-ros2/
     │   │   ├── chapter04-intro-ros2.md
     │   │   ├── chapter05-nodes-topics-services.md
     │   │   ├── chapter06-python-agents-rclpy.md
     │   │   └── chapter07-urdf-modeling.md
     │   ├── module2-digital-twin/
     │   ├── module3-isaac/
     │   └── module4-vla/
     ├── part3-capstone/
     └── part4-future/
     ```
   - Configure sidebar navigation (Docusaurus `sidebars.js`)

2. **Create Chapter Skeletons**:
   - For each of 24 chapters, create file with:
     - Title (H1)
     - Learning Objectives (bullet points — to be filled)
     - Prerequisites (to be filled)
     - Section headings (from chapter template)
     - Summary placeholders
   - Commit skeletons to Git (milestone: "Book structure initialized")

3. **Finalize Tone, Style, and Structure**:
   - Document chapter template (already defined in this plan)
   - Write style guide (tone, voice, jargon policy, accessibility)
   - Create example chapter (e.g., Chapter 1) following template fully
   - Review example chapter against constitution and spec
   - Iterate on template as needed

**Deliverables**:
- ✅ Docusaurus project initialized and building
- ✅ 24 chapter skeleton files created
- ✅ Chapter template documented (in this plan)
- ✅ Example chapter completed (Chapter 1)
- ✅ Style guide documented

---

### Phase 2: Analysis

**Duration**: 3-4 weeks

**Objectives**:
- Break down module content (detailed outlines for Modules 1-4)
- Ensure conceptual alignment across chapters (dependency validation)
- Document decisions with tradeoffs (ADRs for significant decisions)

**Activities**:

1. **Break Down Module Content**:
   - For each module (Modules 1-4):
     - Expand chapter skeletons with detailed section outlines
     - Identify key concepts for each chapter (2-4 concepts per chapter)
     - Determine which diagrams needed (flowcharts, architecture diagrams, sequence diagrams)
     - Assign learning objectives to each chapter
   - Ensure progressive complexity within each module

2. **Ensure Conceptual Alignment Across Chapters**:
   - Create dependency matrix:
     - Rows: Concepts (e.g., "ROS 2 publish-subscribe", "Visual SLAM")
     - Columns: Chapters (Ch 1-24)
     - Cells: Where concept is introduced (I), used (U), or mastered (M)
   - Validate no concept used before introduced
   - Validate capstone chapters use concepts from all modules
   - Adjust chapter content as needed to fix alignment issues

3. **Document Decisions with Tradeoffs**:
   - Identify architecturally significant decisions (per ADR three-part test):
     - Impact: Long-term consequences on book structure
     - Alternatives: Multiple viable options
     - Scope: Cross-cutting effect on multiple chapters
   - Create ADRs for significant decisions (e.g., "ADR-001: Conceptual vs. Practical Code Examples")
   - Store ADRs in `history/adr/`

**Deliverables**:
- ✅ Detailed outlines for all 24 chapters
- ✅ Dependency matrix (concept alignment validated)
- ✅ ADRs for significant decisions (minimum 3-5 ADRs)
- ✅ `data-model.md` (if applicable — book structure entities)
- ✅ `quickstart.md` (reader onboarding guide)

---

### Phase 3: Synthesis

**Duration**: 8-12 weeks (longest phase — actual content writing)

**Objectives**:
- Write final chapters (all 24 chapters)
- Insert diagrams (Mermaid + SVG as needed)
- Perform quality validation (accuracy, readability, accessibility)

**Activities**:

1. **Write Final Chapters**:
   - Follow chapter template structure
   - Write in order (Part 1 → Part 2 → Part 3 → Part 4) to maintain flow
   - Concurrent research for each chapter (validate claims as you write)
   - Target: 2000-4000 words per chapter
   - Commit each completed chapter to Git

2. **Insert Diagrams**:
   - Create Mermaid diagrams for:
     - Flowcharts (conceptual flows)
     - Sequence diagrams (ROS 2 message passing, VLA pipeline)
     - Architecture diagrams (system components)
   - Create SVG diagrams for complex system architectures (e.g., Chapter 20 full integration)
   - Add alt text and figure captions to all diagrams
   - Validate diagram clarity (per Testing & Validation strategy)

3. **Perform Quality Validation**:
   - **Per Chapter** (after each chapter written):
     - Run Flesch-Kincaid readability check
     - Validate accuracy against primary sources
     - Check for broken internal/external links
     - Validate diagram clarity
   - **Per Module** (after each module complete):
     - Validate conceptual alignment within module
     - Ensure skills progression (Ch N prerequisites for Ch N+1)
   - **Book-Wide** (after all chapters complete):
     - Run full link validation
     - Validate capstone integration (backward tracing)
     - Lighthouse audit (performance, accessibility)
     - Beta reader review (if available)

**Deliverables**:
- ✅ 24 completed chapters (all content written)
- ✅ All diagrams created and inserted
- ✅ Quality validation checklists completed for all chapters
- ✅ Beta reader feedback incorporated (if available)

---

### Phase 4: Deployment

**Duration**: 1-2 weeks

**Objectives**:
- Build Docusaurus site (validate build succeeds)
- Deploy to GitHub Pages (configure CI/CD)
- Final validation (Lighthouse, accessibility, cross-browser)

**Activities**:

1. **Build Docusaurus Site**:
   - Run `npm run build` locally
   - Fix any build errors or warnings
   - Verify generated site in `/build` directory
   - Test navigation, search, responsiveness locally

2. **Configure GitHub Pages Deployment**:
   - Create GitHub Actions workflow (`.github/workflows/deploy.yml`)
   - Configure automatic deployment on push to `main` branch
   - Test deployment to GitHub Pages staging URL
   - Verify site loads correctly on public URL

3. **Final Validation**:
   - Run Lighthouse audit (Performance ≥90, Accessibility ≥90)
   - Test on multiple browsers (Chrome, Firefox, Safari, Edge)
   - Test on multiple devices (desktop, tablet, mobile)
   - Run automated link checker (zero broken links)
   - Validate WCAG 2.1 Level AA compliance

4. **Launch Preparation**:
   - Add README.md with book description and link to deployed site
   - Add LICENSE file (choose appropriate license)
   - Add CONTRIBUTING.md (if open to contributions)
   - Create release tag (v1.0.0)

**Deliverables**:
- ✅ Docusaurus site builds with zero errors/warnings
- ✅ GitHub Actions CI/CD workflow configured
- ✅ Site deployed to GitHub Pages
- ✅ Lighthouse scores ≥90 (Performance, Accessibility)
- ✅ Cross-browser/cross-device validation complete
- ✅ Zero broken links
- ✅ README, LICENSE, CONTRIBUTING docs added
- ✅ v1.0.0 release created

---

## Project Structure

### Documentation (this feature)

```text
specs/001-humanoid-robotics-book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (research findings)
├── data-model.md        # Phase 1 output (book structure entities)
├── sources.md           # APA bibliography of primary sources
├── quickstart.md        # Phase 1 output (reader onboarding guide)
├── checklists/
│   └── requirements.md  # Quality validation checklist
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus Static Site Structure

/
├── docs/                           # All book content (Markdown)
│   ├── part1-foundations/
│   │   ├── chapter01-what-is-physical-ai.md
│   │   ├── chapter02-understanding-humanoid-robots.md
│   │   └── chapter03-why-embodied-intelligence-matters.md
│   ├── part2-modules/
│   │   ├── module1-ros2/
│   │   │   ├── chapter04-intro-ros2.md
│   │   │   ├── chapter05-nodes-topics-services.md
│   │   │   ├── chapter06-python-agents-rclpy.md
│   │   │   └── chapter07-urdf-modeling.md
│   │   ├── module2-digital-twin/
│   │   │   ├── chapter08-what-is-digital-twin.md
│   │   │   ├── chapter09-gazebo-simulation.md
│   │   │   ├── chapter10-simulating-sensors.md
│   │   │   └── chapter11-unity-visualization.md
│   │   ├── module3-isaac/
│   │   │   ├── chapter12-isaac-sim-overview.md
│   │   │   ├── chapter13-isaac-ros-perception.md
│   │   │   ├── chapter14-navigation-path-planning.md
│   │   │   └── chapter15-training-humanoid-skills.md
│   │   └── module4-vla/
│   │       ├── chapter16-what-is-vla.md
│   │       ├── chapter17-voice-to-action.md
│   │       ├── chapter18-llms-generate-actions.md
│   │       └── chapter19-capstone-implementation.md
│   ├── part3-capstone/
│   │   ├── chapter20-complete-architecture.md
│   │   ├── chapter21-integrating-all-modules.md
│   │   └── chapter22-final-simulation-walkthrough.md
│   ├── part4-future/
│   │   ├── chapter23-next-decade-embodied-ai.md
│   │   └── chapter24-opportunities-careers.md
│   └── intro.md                    # Book introduction/landing page
│
├── static/                         # Static assets
│   └── img/
│       ├── part1-foundations/      # Images for Part 1 chapters
│       ├── part2-modules/          # Images for Part 2 chapters
│       ├── part3-capstone/         # Images for Part 3 chapters
│       └── part4-future/           # Images for Part 4 chapters
│
├── src/                            # Docusaurus custom components (if needed)
│   └── components/
│
├── docusaurus.config.js            # Docusaurus configuration
├── sidebars.js                     # Sidebar navigation structure
├── package.json                    # Node dependencies (Docusaurus, Mermaid plugin)
├── .github/
│   └── workflows/
│       └── deploy.yml              # GitHub Actions CI/CD for GitHub Pages
├── README.md                       # Project overview
├── LICENSE                         # Book license
└── CONTRIBUTING.md                 # Contribution guidelines (if applicable)
```

**Structure Decision**: Selected **Docusaurus Static Site** structure as it:
- Aligns with Constitution Technical Standards (Docusaurus 3.x, Markdown content)
- Supports version control (all content in Git)
- Enables continuous deployment (GitHub Actions → GitHub Pages)
- Provides excellent navigation (sidebar, search, TOC auto-generated)
- Supports accessibility (Docusaurus has built-in accessibility features)
- Allows Mermaid diagrams (via `@docusaurus/theme-mermaid` plugin)

---

## Complexity Tracking

**No constitutional violations detected.** All constitutional principles aligned with plan (see Constitution Check section above).

This section intentionally left empty as no complexity justifications are needed.

---

## Next Steps

**Phase 0 (Research)** begins immediately. Key outputs:
1. `research.md` — Research findings (technology validation, source gathering)
2. `sources.md` — APA bibliography

After Phase 0 completion, proceed to Phase 1 (Foundation) to initialize Docusaurus project and create chapter skeletons.

**Command to generate tasks**: After plan review and approval, run `/sp.tasks` to break down phases into actionable tasks.
