---
id: 014
title: ADR Review - Curriculum Architecture Decisions
stage: plan
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-curriculum-restructure
branch: 001-curriculum-restructure
user: system
command: /sp.adr
labels: ["adr", "architecture", "curriculum-design", "pedagogical-framework"]
links:
  spec: specs/001-curriculum-restructure/spec.md
  ticket: null
  adr: history/adr/004-curriculum-structure-module-based-organization.md, history/adr/005-pedagogical-framework-blooms-taxonomy.md
  pr: null
files:
  - history/adr/004-curriculum-structure-module-based-organization.md
  - history/adr/005-pedagogical-framework-blooms-taxonomy.md
tests:
---

## Prompt

/sp.adr

(No additional arguments provided - system-initiated ADR review after planning phase completion)

## Response snapshot

Analyzed planning artifacts (plan.md, research.md, data-model.md) and identified 2 architecturally significant decision clusters:

**ADR-004: Curriculum Structure - Module-Based Organization**
- Decision: Adopt 4-module, 13-week structure replacing chapter-based organization (supersedes ADR-003 for curriculum content)
- Components: Module folders (module-01-ros2/, module-02-gazebo-unity/, module-03-nvidia-isaac/, module-04-vla-conversational/), week-based content files, capstone/ folder
- Rationale: Provides clear learning timeline, thematic clustering by technology stack, explicit prerequisite relationships
- Alternatives: Chapter-based (current ADR-003), flat weekly structure, spiral curriculum
- Key Trade-off: Breaking change requiring content migration vs. pedagogically superior structure

**ADR-005: Pedagogical Framework - Bloom's Taxonomy Progression**
- Decision: Use Bloom's Taxonomy (Remember → Understand → Apply → Analyze → Evaluate → Create) for 13-week learning progression
- Components: Cognitive level mapping (Weeks 1-2: Remember/Understand, Weeks 3-5: Apply, etc.), prerequisite enforcement, complexity progression in hands-on exercises
- Rationale: Research-backed pedagogical model, clear skill development trajectory, measurable learning outcomes
- Alternatives: Problem-based learning, spiral curriculum, competency-based progression, constructivist learning
- Key Trade-off: Rigid sequential progression vs. proven skill-building framework

Both ADRs passed significance test:
- Impact: Long-term consequences for curriculum structure and content authoring (YES)
- Alternatives: Multiple viable options considered with explicit trade-offs (YES)
- Scope: Cross-cutting changes affecting navigation, metadata, and all future content (YES)

Existing ADRs reviewed:
- ADR-001: Documentation Platform Stack (no conflict - infrastructure decision)
- ADR-002: Testing and Quality Strategy (no conflict - quality assurance decision)
- ADR-003: Content Architecture and Metadata (SUPERSEDED for curriculum by ADR-004, but still applies to general documentation)

## Outcome

- ✅ Impact: Created 2 comprehensive ADRs documenting curriculum restructure architectural decisions
- 🧪 Tests: No tests run (ADR creation is documentation task)
- 📁 Files: 2 new ADR files created in history/adr/
- 🔁 Next prompts: Run /sp.tasks to generate implementation tasks based on plan.md and ADR decisions
- 🧠 Reflection: ADR clustering strategy worked well - grouped "module structure" and "pedagogical framework" as separate decision clusters rather than over-granular technology choices. Both decisions met all 3 significance criteria (impact, alternatives, scope).

## Evaluation notes (flywheel)

- Failure modes observed: None - ADR creation followed template structure, all placeholders filled, decision clusters appropriately scoped
- Graders run and results (PASS/FAIL):
  - Clustered decisions (not atomic): PASS - Grouped related organizational/pedagogical choices
  - Explicit alternatives listed: PASS - Each ADR documents 3-4 alternatives with rationale for rejection
  - Consequences cover positive/negative: PASS - Both ADRs list pros, cons, and mitigation strategies
  - References link to plan/research: PASS - All ADRs reference plan.md, research.md, data-model.md, spec.md
- Prompt variant (if applicable): Standard /sp.adr command (no variant)
- Next experiment (smallest change to try): Consider adding "supersedes" field to ADR frontmatter to explicitly track ADR evolution (ADR-004 supersedes ADR-003 for curriculum, but this relationship only mentioned in prose)
