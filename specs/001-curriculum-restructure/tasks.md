---
description: "Task list for Physical AI Book Curriculum Restructure"
---

# Tasks: Physical AI Book Curriculum Restructure

**Input**: Design documents from `/specs/001-curriculum-restructure/`
**Prerequisites**: plan.md (tech stack, structure), spec.md (user stories), research.md (pedagogy), data-model.md (entities), quickstart.md (authoring guide)

**Feature Branch**: `001-curriculum-restructure`
**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

**Tests**: No automated tests required - content validation is editorial review per plan.md (Constitution Check: Principle VII)

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Project Initialization)

**Purpose**: Prepare documentation structure and establish authoring conventions

- [X] T001 Remove existing placeholder chapter folders (docs/chapter-01-foundations/, docs/chapter-02-mechanics/) per plan.md
- [X] T002 [P] Create Module 1 folder structure (docs/module-01-ros2/) with index.md entry point
- [X] T003 [P] Create Module 2 folder structure (docs/module-02-gazebo-unity/) with index.md entry point
- [X] T004 [P] Create Module 3 folder structure (docs/module-03-nvidia-isaac/) with index.md entry point
- [X] T005 [P] Create Module 4 folder structure (docs/module-04-vla-conversational/) with index.md entry point
- [X] T006 Create Capstone folder structure (docs/capstone/) with index.md, requirements.md, integration-guide.md per plan.md

**Checkpoint**: ✅ Documentation structure ready for content authoring

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core curriculum structure that MUST be complete before ANY user story content implementation

**⚠️ CRITICAL**: No user story content work can begin until module overviews are complete

- [X] T007 Write Module 1 overview (docs/module-01-ros2/index.md) with learning objectives, prerequisites, and weeks 1-5 summary per data-model.md
- [X] T008 Write Module 2 overview (docs/module-02-gazebo-unity/index.md) with learning objectives, prerequisites, and weeks 6-7 summary per data-model.md
- [X] T009 Write Module 3 overview (docs/module-03-nvidia-isaac/index.md) with learning objectives, prerequisites, and weeks 8-10 summary per data-model.md
- [X] T010 Write Module 4 overview (docs/module-04-vla-conversational/index.md) with learning objectives, prerequisites, and weeks 11-13 summary per data-model.md
- [X] T011 Update docs/intro.md to reference new 4-module, 13-week curriculum structure with navigation links

**Checkpoint**: ✅ Foundation ready - user story content implementation can now begin in parallel

---

## Phase 3: User Story 1 - Learner Follows Progressive 13-Week Curriculum (Priority: P1) 🎯 MVP

**Goal**: Deliver complete 13-week progressive learning path from foundational concepts through autonomous humanoid capstone

**Independent Test**: Learner can navigate Week 1-13 content sequentially, complete hands-on exercises, and build capstone project independently per spec.md acceptance scenarios

### Weeks 1-2: Introduction to Physical AI (Foundation)

- [ ] T012 [P] [US1] Write Week 1 content (docs/module-01-ros2/week-01-intro-physical-ai.md) covering Physical AI foundations, embodied intelligence intro, humanoid robotics landscape per FR-004
- [ ] T013 [P] [US1] Write Week 2 content (docs/module-01-ros2/week-02-embodied-intelligence.md) covering embodied intelligence theory, sensor systems overview per FR-004
- [ ] T014 [US1] Add hands-on exercise to Week 1: "Explore Physical AI use cases" with estimation time 30 minutes per FR-015, quickstart.md
- [ ] T015 [US1] Add hands-on exercise to Week 2: "Compare sensor types for humanoid robots" with estimation time 45 minutes per FR-015

### Weeks 3-5: Module 1 - ROS 2 Fundamentals

- [ ] T016 [P] [US1] Write Week 3 content (docs/module-01-ros2/week-03-ros2-architecture.md) covering ROS 2 architecture, DDS middleware, nodes/topics/services/actions per FR-005
- [ ] T017 [P] [US1] Write Week 4 content (docs/module-01-ros2/week-04-ros2-nodes-topics.md) covering node creation, topic publishing/subscribing, services, actions per FR-005
- [ ] T018 [P] [US1] Write Week 5 content (docs/module-01-ros2/week-05-ros2-python-urdf.md) covering rclpy Python packages, URDF for humanoids per FR-005, FR-010
- [ ] T019 [US1] Add hands-on exercise to Week 3: "Inspect ROS 2 nodes using CLI tools" with inline code snippet (15-30 lines) per FR-015a, research.md
- [ ] T020 [US1] Add hands-on exercise to Week 4: "Create minimal ROS 2 publisher node in Python" with GitHub repo link per FR-015a, research.md
- [ ] T021 [US1] Add hands-on exercise to Week 5: "Define humanoid robot URDF model" with GitHub repo link per FR-015

### Weeks 6-7: Module 2 - Gazebo/Unity Simulation

- [ ] T022 [P] [US1] Write Week 6 content (docs/module-02-gazebo-unity/week-06-gazebo-fundamentals.md) covering Gazebo setup, URDF/SDF formats, physics simulation (gravity, collisions) per FR-006, FR-011
- [ ] T023 [P] [US1] Write Week 7 content (docs/module-02-gazebo-unity/week-07-unity-sensors.md) covering Unity introduction, sensor simulation (LiDAR, depth cameras, IMUs), HRI rendering per FR-006, FR-011
- [ ] T024 [US1] Add hands-on exercise to Week 6: "Launch humanoid robot in Gazebo environment" with GitHub repo link per FR-015
- [ ] T025 [US1] Add hands-on exercise to Week 7: "Simulate depth camera in Unity" with GitHub repo link per FR-015

### Weeks 8-10: Module 3 - NVIDIA Isaac Platform

- [ ] T026 [P] [US1] Write Week 8 content (docs/module-03-nvidia-isaac/week-08-isaac-sdk-sim.md) covering Isaac SDK, Isaac Sim, GPU requirements and cloud alternatives (Colab, Paperspace, SageMaker) per FR-007, FR-012a, research.md
- [ ] T027 [P] [US1] Write Week 9 content (docs/module-03-nvidia-isaac/week-09-ai-perception.md) covering AI-powered perception, manipulation, Isaac ROS VSLAM per FR-007, FR-012
- [ ] T028 [P] [US1] Write Week 10 content (docs/module-03-nvidia-isaac/week-10-reinforcement-learning.md) covering reinforcement learning, Nav2 path planning for bipedal movement, sim-to-real transfer per FR-007, FR-012
- [ ] T029 [US1] Add hands-on exercise to Week 8: "Setup Isaac Sim in Google Colab with GPU" with step-by-step cloud setup per FR-012a, FR-015, research.md
- [ ] T030 [US1] Add hands-on exercise to Week 9: "Implement object detection with Isaac ROS" with GitHub repo link per FR-015
- [ ] T031 [US1] Add hands-on exercise to Week 10: "Train RL agent for bipedal locomotion" with GitHub repo link per FR-015

### Weeks 11-13: Module 4 - Humanoid Development & Conversational Robotics

- [ ] T032 [P] [US1] Write Week 11 content (docs/module-04-vla-conversational/week-11-humanoid-kinematics.md) covering kinematics/dynamics, bipedal locomotion basics per FR-008
- [ ] T033 [P] [US1] Write Week 12 content (docs/module-04-vla-conversational/week-12-bipedal-locomotion.md) covering advanced bipedal locomotion, manipulation/grasping, human-robot interaction per FR-008
- [ ] T034 [P] [US1] Write Week 13 content (docs/module-04-vla-conversational/week-13-conversational-robotics.md) covering OpenAI Whisper integration, GPT voice-to-action, natural language to ROS 2 actions, multi-modal interaction per FR-009, FR-013
- [ ] T035 [US1] Add hands-on exercise to Week 11: "Calculate forward kinematics for humanoid arm" with inline code snippet per FR-015
- [ ] T036 [US1] Add hands-on exercise to Week 12: "Implement grasping controller with MoveIt" with GitHub repo link per FR-015
- [ ] T037 [US1] Add hands-on exercise to Week 13: "Build voice command interface with Whisper" with GitHub repo link per FR-015

### Capstone Project Integration

- [ ] T038 [US1] Write Capstone overview (docs/capstone/index.md) describing autonomous humanoid project integrating all 4 modules per FR-014, FR-019
- [ ] T039 [US1] Write Capstone requirements (docs/capstone/requirements.md) detailing voice commands, path planning, object identification, manipulation per FR-014, data-model.md
- [ ] T040 [US1] Write Capstone integration guide (docs/capstone/integration-guide.md) showing how Week 1-13 concepts combine into final project per FR-019, data-model.md
- [ ] T041 [US1] Add cross-references in Week 13 to Capstone project with explicit concept mapping per FR-019

**Checkpoint**: User Story 1 complete - Full 13-week curriculum implemented with hands-on exercises and capstone integration

---

## Phase 4: User Story 2 - Educator Plans Course Using Book Structure (Priority: P2)

**Goal**: Enable instructors to design semester-long Physical AI courses using clear module structure, learning objectives, and assessment points

**Independent Test**: Instructor can review book structure, create course syllabus based on modules, and identify assessment points per spec.md acceptance scenarios

### Module Learning Objectives & Prerequisites

- [ ] T042 [P] [US2] Enhance Module 1 overview with explicit course outcome mappings, prerequisite checklist (basic Python, CLI) per FR-003a, spec.md US2
- [ ] T043 [P] [US2] Enhance Module 2 overview with prerequisite references to Module 1 concepts (ROS 2 nodes, topics) per FR-018, spec.md US2
- [ ] T044 [P] [US2] Enhance Module 3 overview with prerequisite references to Modules 1-2 (ROS 2 + simulation) per FR-018, spec.md US2
- [ ] T045 [P] [US2] Enhance Module 4 overview with prerequisite references to Modules 1-3 (ROS 2 + simulation + AI) per FR-018, spec.md US2

### Assessment & Evaluation Points

- [ ] T046 [US2] Add "Assessment Checkpoints" section to Module 1 overview identifying evaluation points after Weeks 3, 5 per spec.md US2
- [ ] T047 [US2] Add "Assessment Checkpoints" section to Module 2 overview identifying evaluation points after Week 7 per spec.md US2
- [ ] T048 [US2] Add "Assessment Checkpoints" section to Module 3 overview identifying evaluation points after Week 10 per spec.md US2
- [ ] T049 [US2] Add "Assessment Checkpoints" section to Module 4 overview identifying evaluation points after Week 13 per spec.md US2
- [ ] T050 [US2] Add evaluation criteria section to Capstone requirements (docs/capstone/requirements.md) with success metrics per data-model.md, spec.md US2

### Course Planning Resources

- [ ] T051 [US2] Create "For Educators" section in docs/intro.md with semester mapping guidance (13 weeks → 15-week semester) per spec.md US2
- [ ] T052 [US2] Add estimated time metadata to each week's frontmatter (reading + exercise time) per data-model.md, quickstart.md, spec.md US2

**Checkpoint**: User Story 2 complete - Instructors have clear course planning structure with learning objectives, prerequisites, and assessment points

---

## Phase 5: User Story 3 - Self-Directed Learner Accesses Module Reference (Priority: P3)

**Goal**: Enable practitioners with partial robotics knowledge to jump directly to specific modules with clear cross-references to earlier content

**Independent Test**: Practitioner can start at Module 3 and find clear references to Modules 1-2 concepts when needed per spec.md acceptance scenarios

### Cross-References & Navigation

- [ ] T053 [P] [US3] Add "Quick Prerequisites" section to Module 3 overview linking to specific Module 1-2 concepts (ROS 2 nodes, Gazebo setup) per spec.md US3
- [ ] T054 [P] [US3] Add "Quick Prerequisites" section to Module 4 overview linking to specific Module 1-3 concepts (ROS 2 actions, Isaac Sim, Nav2) per spec.md US3
- [ ] T055 [US3] Add inline cross-references in Week 8 content linking to Week 3 (ROS 2 architecture) and Week 6 (Gazebo fundamentals) per spec.md US3
- [ ] T056 [US3] Add inline cross-references in Week 9 content linking to Week 4 (ROS 2 topics) and Week 7 (sensor simulation) per spec.md US3
- [ ] T057 [US3] Add inline cross-references in Week 11-13 content linking to earlier weeks' key concepts per spec.md US3

### Modular Content Structure

- [ ] T058 [US3] Add "Prerequisites Recap" subsection to Week 8 summarizing required ROS 2 and simulation knowledge per spec.md US3
- [ ] T059 [US3] Add "Prerequisites Recap" subsection to Week 11 summarizing required knowledge from Modules 1-3 per spec.md US3
- [ ] T060 [US3] Ensure all module overviews include "Can I skip earlier modules?" guidance per spec.md US3

**Checkpoint**: User Story 3 complete - Self-directed learners can navigate flexibly with clear cross-references

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and overall content quality

### Content Quality & Validation

- [ ] T061 [P] Verify all 13 weeks include YAML frontmatter with title, sidebar_position, week_number, module, learning_objectives per FR-021, data-model.md
- [ ] T062 [P] Verify all 4 module overviews include YAML frontmatter with module_id, module_number, weeks_included, learning_objectives per data-model.md
- [ ] T063 Validate sidebar_position values are sequential and unique within each module per FR-024, quickstart.md validation checklist
- [ ] T064 Validate all hands-on exercises include title, description, estimated_time, tools_required per data-model.md, FR-015
- [ ] T065 Verify prerequisites only reference earlier weeks/modules (no forward references) per data-model.md validation rules

### Technical Accuracy & Links

- [ ] T066 [P] Verify all ROS 2 references are accurate for ROS 2 Humble distribution per FR-005
- [ ] T067 [P] Verify all Gazebo/Unity references are current (Gazebo Fortress, Unity 2022.3 LTS) per FR-006
- [ ] T068 [P] Verify all NVIDIA Isaac references are accurate for Isaac Sim 2023.1 per FR-007
- [ ] T069 [P] Verify all OpenAI Whisper/GPT integration references are current per FR-009
- [ ] T070 Test all internal cross-references (week-to-week, module-to-module links) using `npm run build` with onBrokenLinks: 'throw' per plan.md, quickstart.md

### Search & Discoverability

- [ ] T071 [P] Add keywords array to all week frontmatter for Algolia DocSearch indexing per data-model.md, FR-022b
- [ ] T072 [P] Add keywords array to all module frontmatter for search indexing per data-model.md
- [ ] T073 Verify all learning objectives use Bloom's Taxonomy action verbs (identify, implement, compare, evaluate, design) per research.md, quickstart.md

### GitHub & External Resources

- [ ] T074 [P] Create placeholder GitHub repository references for complete code projects in weeks needing external repos per FR-015a, research.md
- [ ] T075 [P] Add cloud GPU setup instructions (Colab, Paperspace, SageMaker) to Week 8 content per FR-012a, research.md
- [ ] T076 Ensure glossary.md (docs/glossary.md) includes key terms from all 4 modules per plan.md

### Build & Deployment Validation

- [ ] T077 Run `npm run build` to verify no broken links or build errors per plan.md Constitution Check Principle III
- [ ] T078 Run `npm run start` and manually navigate through all 4 modules and 13 weeks per quickstart.md validation checklist
- [ ] T079 Verify autogenerated sidebars display modules as categories with sequential week ordering per FR-022, FR-024
- [ ] T080 Run existing Playwright tests (tests/navigation.test.js, tests/links.test.js, tests/responsive.test.js) to validate new structure per plan.md Constitution Check Principle VII

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after US1 core content complete (T012-T041) - Enhances existing content
  - User Story 3 (P3): Can start after US1 core content complete (T012-T041) - Adds cross-references
- **Polish (Phase 6)**: Depends on all user stories being complete

### Within Each User Story

- **User Story 1**: Sequential week dependencies (Week 1 before Week 2, etc.) but parallel within week groups
- **User Story 2**: Depends on module overviews from US1, then all enhancement tasks can run in parallel
- **User Story 3**: Depends on week content from US1, then all cross-reference tasks can run in parallel

### Parallel Opportunities

- All Setup tasks (T002-T005) can run in parallel (different module folders)
- Week authoring within the same module range can run in parallel:
  - Weeks 1-2 (T012-T013) - parallel
  - Weeks 3-5 (T016-T018) - parallel
  - Weeks 6-7 (T022-T023) - parallel
  - Weeks 8-10 (T026-T028) - parallel
  - Weeks 11-13 (T032-T034) - parallel
- All US2 enhancement tasks (T042-T045, module prerequisites) - parallel
- All US3 cross-reference tasks (T053-T054) - parallel
- All validation tasks in Polish phase can run in parallel

---

## Parallel Example: User Story 1 Week Authoring

```bash
# Launch Weeks 3-5 authoring together (Module 1 ROS 2 content):
Task: "Write Week 3 content (docs/module-01-ros2/week-03-ros2-architecture.md)"
Task: "Write Week 4 content (docs/module-01-ros2/week-04-ros2-nodes-topics.md)"
Task: "Write Week 5 content (docs/module-01-ros2/week-05-ros2-python-urdf.md)"

# Launch Weeks 8-10 authoring together (Module 3 Isaac content):
Task: "Write Week 8 content (docs/module-03-nvidia-isaac/week-08-isaac-sdk-sim.md)"
Task: "Write Week 9 content (docs/module-03-nvidia-isaac/week-09-ai-perception.md)"
Task: "Write Week 10 content (docs/module-03-nvidia-isaac/week-10-reinforcement-learning.md)"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Full Curriculum)

1. Complete Phase 1: Setup (T001-T006) - Documentation structure
2. Complete Phase 2: Foundational (T007-T011) - Module overviews
3. Complete Phase 3: User Story 1 (T012-T041) - All 13 weeks + capstone
4. **STOP and VALIDATE**: Manual review of full curriculum, test navigation, verify all acceptance scenarios
5. Deploy and gather learner feedback

**MVP Delivers**: Complete 13-week progressive learning path with hands-on exercises and capstone project (SC-001, SC-003, SC-005, SC-010 from spec.md)

### Incremental Delivery

1. Complete Setup + Foundational → Module structure ready
2. Add User Story 1 → Test 13-week curriculum → Deploy/Demo (MVP!)
3. Add User Story 2 → Test educator planning scenarios → Deploy/Demo
4. Add User Story 3 → Test flexible navigation → Deploy/Demo
5. Complete Polish phase → Final validation and optimization

### Parallel Team Strategy

With multiple content authors:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Author A: Weeks 1-5 (Introduction + ROS 2)
   - Author B: Weeks 6-10 (Simulation + Isaac)
   - Author C: Weeks 11-13 + Capstone (Humanoid + VLA)
3. All authors use templates from quickstart.md for consistency
4. Team lead validates frontmatter and cross-references

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- All content follows templates in quickstart.md
- Use data-model.md for entity structure and frontmatter requirements
- Follow research.md for pedagogical best practices (Bloom's Taxonomy, hybrid code examples)
- Verify all tasks meet functional requirements (FR-001 through FR-024) from spec.md
- No automated test generation required - content validation is editorial review
- Commit after each week or logical group (e.g., all Weeks 3-5 together)
- Stop at any checkpoint to validate story independently
