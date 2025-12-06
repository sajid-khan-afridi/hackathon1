---
id: 001
title: Generate curriculum restructure tasks
stage: tasks
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5
feature: 001-curriculum-restructure
branch: 001-curriculum-restructure
user: agent
command: /sp.tasks
labels: ["task-generation", "curriculum", "content-authoring"]
links:
  spec: specs/001-curriculum-restructure/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-curriculum-restructure/tasks.md
tests:
 - null
---

## Prompt

/sp.tasks

Context: Generate actionable, dependency-ordered tasks.md for the Physical AI Book Curriculum Restructure feature based on available design artifacts (spec.md, plan.md, research.md, data-model.md, quickstart.md).

## Response snapshot

Successfully generated comprehensive tasks.md with 80 tasks organized by user story priority:

**Task Organization:**
- Phase 1: Setup (T001-T006) - Documentation structure and module folder creation
- Phase 2: Foundational (T007-T011) - Module overviews and core structure (BLOCKS all user stories)
- Phase 3: User Story 1 (T012-T041) - Complete 13-week progressive curriculum with hands-on exercises and capstone (Priority P1, MVP)
- Phase 4: User Story 2 (T042-T052) - Educator course planning enhancements (Priority P2)
- Phase 5: User Story 3 (T053-T060) - Self-directed learner cross-references (Priority P3)
- Phase 6: Polish (T061-T080) - Content validation, technical accuracy, build verification

**Key Features:**
- All tasks follow strict checklist format: `- [ ] [TID] [P?] [Story?] Description with file path`
- Story labels: [US1], [US2], [US3] for traceability
- Parallelization markers [P] for concurrent execution opportunities
- Independent test criteria for each user story
- Clear dependencies and execution order
- Parallel execution examples for team coordination
- MVP strategy (User Story 1 = complete 13-week curriculum)

**Parallel Opportunities Identified:**
- Module folder creation (4 modules simultaneously)
- Week authoring within module ranges (Weeks 3-5, 6-7, 8-10, 11-13)
- User Story 2 enhancement tasks (module prerequisites)
- User Story 3 cross-reference tasks
- Validation tasks in Polish phase

**Mapping to Requirements:**
- All functional requirements FR-001 through FR-024 mapped to specific tasks
- Each user story from spec.md has dedicated phase with acceptance scenarios
- Pedagogical best practices from research.md integrated (Bloom's Taxonomy, hybrid code examples)
- Data model entities reflected in content structure
- Quickstart.md authoring guidelines referenced throughout

## Outcome

- ✅ Impact: 80 executable tasks created, organized by 3 user stories for independent implementation and MVP delivery
- 🧪 Tests: No automated tests (content validation is editorial review per plan.md Constitution Check)
- 📁 Files: Created specs/001-curriculum-restructure/tasks.md (comprehensive task list)
- 🔁 Next prompts: Execute /sp.implement to begin task implementation starting with Phase 1 Setup
- 🧠 Reflection: Task organization by user story enables incremental delivery - User Story 1 alone delivers complete MVP (13-week curriculum), while US2/US3 add educator and self-directed learner value

## Evaluation notes (flywheel)

- Failure modes observed: None - all design documents were complete and well-structured
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks follow checkbox + ID + [P?] + [Story?] + description pattern)
- Prompt variant (if applicable): Standard /sp.tasks command workflow
- Next experiment (smallest change to try): Monitor task execution for any missing dependencies or unclear acceptance criteria
