# Humonide Robot Book Project Constitution
<!-- AI/Spec-Driven Book Creation using Docusaurus and GitHub Pages -->

<!--
SYNC IMPACT REPORT:
Version: 0.0.0 → 1.0.0 (Initial constitution)
Modified Principles: N/A (Initial creation)
Added Sections:
  - Core Principles (6 principles defined)
  - Content Standards
  - Technical Standards
  - Governance
Templates Status:
  ✅ Constitution created from template
  ⚠ plan-template.md - Review alignment with principles
  ⚠ spec-template.md - Review alignment with content standards
  ⚠ tasks-template.md - Review alignment with workflow
Follow-up TODOs: None
-->

## Core Principles

### I. Content Accuracy and Verification
Every factual claim, code example, and technical explanation in the book MUST be verifiable and accurate.

- All technical content must be tested and validated before inclusion
- Code examples MUST be executable and produce stated results
- Third-party tools, libraries, and services MUST be referenced with correct versions and documentation links
- Claims about AI capabilities, Spec-Kit Plus features, or Claude Code functionality MUST be verified against official documentation or direct testing
- Any assumptions or opinions MUST be clearly labeled as such

**Rationale**: Readers trust technical books to provide accurate information. Inaccurate content damages credibility and wastes reader time. For AI/developer tools, accuracy is critical as readers will use this content to build real projects.

### II. Clarity for Target Audience
Content MUST be written for developers and technical writers with basic knowledge of modern web development, Git, and command-line tools.

- Assume reader familiarity with: JavaScript/TypeScript, Node.js, Git basics, Markdown
- Explain specialized concepts: Spec-Driven Development, AI-assisted development workflows, Docusaurus-specific features
- Use progressive disclosure: simple concepts first, advanced topics later
- Include visual aids (diagrams, screenshots, code blocks) where they improve understanding
- Writing clarity target: Flesch-Kincaid grade level 10-12

**Rationale**: Books that assume too much knowledge alienate beginners; books that explain too much bore experienced readers. Clear audience definition ensures appropriate depth and pacing.

### III. Reproducibility (NON-NEGOTIABLE)
Every tutorial, example, and workflow in the book MUST be reproducible by readers following the documented steps.

- Step-by-step instructions MUST include all prerequisites
- Environment setup MUST specify exact versions of tools when version-specific
- Code examples MUST be complete (no "..." placeholders without explanation)
- Expected outputs MUST be shown for key steps
- Troubleshooting sections MUST address common failure modes
- All example repositories and live demos MUST remain accessible

**Rationale**: A technical book's value is measured by whether readers can successfully apply what they learn. Non-reproducible examples frustrate readers and undermine the book's purpose.

### IV. Spec-Driven Development Integration
All book content creation MUST follow Spec-Kit Plus methodology and be traceable through specs, plans, and tasks.

- Each chapter/section MUST have a corresponding spec defining scope and learning objectives
- Content changes MUST follow spec → plan → tasks → implementation workflow
- Major structural decisions MUST be documented as ADRs
- Prompt History Records (PHRs) MUST capture all AI-assisted content creation sessions
- Constitution principles MUST guide all content decisions

**Rationale**: The book teaches Spec-Driven Development; it must be written using the same methodology to demonstrate real-world application and ensure methodical, high-quality content creation.

### V. Version Control and Collaboration
All book content, code examples, and assets MUST be version-controlled and support collaborative review.

- Git MUST be used for all content versioning
- Commits MUST be atomic and have clear, descriptive messages
- Branch naming MUST follow convention: `chapter/<chapter-name>` or `feature/<feature-name>`
- Pull requests MUST be used for significant content additions or revisions
- Docusaurus site MUST build successfully before merging content changes

**Rationale**: Version control enables collaboration, tracks content evolution, supports rollback, and integrates with deployment pipelines. For a book about modern development, proper version control is non-negotiable.

### VI. Deployment and Accessibility
The book MUST be continuously deployable to GitHub Pages and accessible to readers at all times.

- Docusaurus build MUST pass without errors before deployment
- GitHub Pages deployment MUST be automated via GitHub Actions
- Site MUST be responsive and accessible (WCAG 2.1 Level AA minimum)
- Navigation MUST be intuitive (clear table of contents, search functionality)
- Performance MUST be acceptable (Lighthouse score ≥ 90 for Performance and Accessibility)
- Site MUST include clear license and contribution guidelines

**Rationale**: A book about modern web development should exemplify best practices in deployment, accessibility, and user experience. Continuous deployment ensures readers always access the latest content.

## Content Standards

### Scope and Structure
- **Book Focus**: Teaching AI/Spec-Driven Development for book creation using Spec-Kit Plus and Claude Code
- **Primary Audience**: Developers and technical writers comfortable with web development basics
- **Format**: Docusaurus-based static site deployed to GitHub Pages
- **Chapter Organization**: Progressive from setup → core concepts → advanced workflows → deployment
- **Example Depth**: Balance between concise demonstrations and real-world complexity

### Citation and Attribution
- External tools and libraries MUST link to official documentation
- Code snippets adapted from external sources MUST include attribution
- Screenshots of third-party tools MUST be used fairly (educational use)
- Spec-Kit Plus and Claude Code capabilities MUST reference official documentation

### Quality Gates
- All code examples MUST pass linting (ESLint/Prettier as configured)
- Markdown MUST be validated for broken links before merge
- New chapters MUST be reviewed by at least one other contributor (if team grows)
- Docusaurus build MUST succeed with zero warnings

## Technical Standards

### Technology Stack
- **Static Site Generator**: Docusaurus 3.x (latest stable)
- **Content Format**: Markdown (.md or .mdx for interactive components)
- **Version Control**: Git + GitHub
- **Deployment**: GitHub Pages via GitHub Actions
- **Node.js**: LTS version (currently 20.x recommended)
- **Package Manager**: npm or yarn (specify in project docs)

### Development Workflow
1. **Content Planning**: Create spec for new chapter/section
2. **Task Breakdown**: Generate tasks.md with specific writing and validation tasks
3. **Draft Creation**: Write content using Claude Code assistance, following spec
4. **Local Testing**: Build Docusaurus site locally and review
5. **Review**: Self-review or peer review against constitution and spec
6. **Deployment**: Merge to main triggers automatic GitHub Pages deployment

### Code Examples Policy
- Inline code: use `backticks` for commands, file names, short code
- Code blocks: use fenced code blocks with language specification
- Longer examples: link to separate files in `/examples` directory or companion repository
- All code examples MUST include comments explaining non-obvious logic

### Asset Management
- Images: Store in `/static/img/<chapter-name>/`
- Diagrams: Prefer Mermaid diagrams (as code) over static images when feasible
- File size: Optimize images (<500KB per image recommended)
- Alt text: MUST be provided for all images (accessibility requirement)

## Governance

### Amendment Process
This constitution governs all content and technical decisions for the book project. Amendments follow this process:

1. **Proposal**: Document proposed change with rationale (via issue or discussion)
2. **Impact Analysis**: Assess effect on existing content, templates, and workflows
3. **Approval**: Self-approval for solo author; consensus for team projects
4. **Migration Plan**: Update affected specs, plans, tasks, and content
5. **Version Bump**: Update version number per semantic versioning rules below
6. **Documentation**: Update Sync Impact Report at top of constitution

### Versioning Policy
- **MAJOR** (X.0.0): Backward-incompatible changes (e.g., removing a core principle, changing target audience significantly)
- **MINOR** (x.Y.0): Additions or expansions (e.g., new principle added, new section added)
- **PATCH** (x.y.Z): Clarifications, wording improvements, typo fixes

### Compliance Review
- All PRs MUST verify alignment with constitution principles
- PHRs MUST reference relevant principles when making content decisions
- ADRs MUST be created for decisions meeting the three-part significance test:
  - **Impact**: Long-term consequences on book structure, workflow, or reader experience
  - **Alternatives**: Multiple viable options with different tradeoffs
  - **Scope**: Cross-cutting effect on multiple chapters or the overall book architecture

### Conflict Resolution
When principles appear to conflict (e.g., Clarity vs. Accuracy), prioritize in this order:
1. Accuracy (never sacrifice correctness for simplicity)
2. Reproducibility (readers must be able to follow along)
3. Clarity (explain complexity, don't hide it)
4. Spec-Driven methodology adherence
5. Deployment and accessibility standards

**Version**: 1.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
