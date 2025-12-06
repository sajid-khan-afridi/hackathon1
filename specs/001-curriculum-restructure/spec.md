# Feature Specification: Physical AI Book Curriculum Restructure

**Feature Branch**: `001-curriculum-restructure`
**Created**: 2025-12-04
**Status**: Draft
**Input**: User description: "Restructure book content to match finalized 4-module curriculum: Module 1 (ROS 2), Module 2 (Gazebo/Unity), Module 3 (NVIDIA Isaac), Module 4 (VLA). Replace placeholder chapters with 13-week progressive learning path including hands-on projects and autonomous humanoid capstone."

## User Scenarios & Testing

### User Story 1 - Learner Follows Progressive 13-Week Curriculum (Priority: P1)

A student opens the Physical AI book and follows the 13-week structured learning path from foundational concepts through advanced autonomous robotics, completing hands-on projects at each module.

**Why this priority**: This is the core value proposition - delivering a complete, progressive learning experience that takes learners from basics to building autonomous humanoid robots.

**Independent Test**: Can be fully tested by following Week 1-2 content through to Week 13, completing exercises, and building the capstone project. Success means a learner can complete the autonomous humanoid project independently.

**Acceptance Scenarios**:

1. **Given** a learner opens the book, **When** they navigate to the table of contents, **Then** they see 4 clearly defined modules with 13 weeks of content organized sequentially
2. **Given** a learner completes Module 1 (ROS 2), **When** they start Module 2, **Then** they can apply ROS 2 knowledge to the Gazebo/Unity simulations
3. **Given** a learner reaches Week 13, **When** they review the capstone project requirements, **Then** they can identify how all previous modules integrate into the final autonomous humanoid
4. **Given** a learner completes any week, **When** they look for hands-on activities, **Then** they find practical exercises that reinforce the week's concepts

---

### User Story 2 - Educator Plans Course Using Book Structure (Priority: P2)

An instructor uses the book to design a semester-long Physical AI course, mapping the 13-week structure to their academic calendar and using the module breakdown for lesson planning.

**Why this priority**: Educators are key multipliers who will use this book with multiple students. Clear structure enables course adoption.

**Independent Test**: Can be tested by an instructor reviewing the book structure, creating a course syllabus based on the modules, and identifying assessment points. Success means the book provides sufficient scaffolding for course creation.

**Acceptance Scenarios**:

1. **Given** an instructor reviews the curriculum, **When** they examine module learning objectives, **Then** they can map each module to specific course outcomes
2. **Given** an instructor plans assessments, **When** they review each module's projects, **Then** they can identify evaluation points for student progress
3. **Given** an instructor needs prerequisite information, **When** they review Weeks 1-2, **Then** they understand what foundational knowledge students need before starting

---

### User Story 3 - Self-Directed Learner Accesses Module Reference (Priority: P3)

A practitioner with partial robotics knowledge jumps directly to Module 3 (NVIDIA Isaac) to learn specific AI-robot integration techniques, using earlier modules as reference when needed.

**Why this priority**: Enables flexible learning paths for experienced practitioners who don't need to follow the full 13-week sequence.

**Independent Test**: Can be tested by jumping to Module 3 content directly and verifying that cross-references to earlier modules are clear and accessible.

**Acceptance Scenarios**:

1. **Given** a practitioner starts at Module 3, **When** they encounter ROS 2 concepts, **Then** they find clear references to Module 1 content for review
2. **Given** a practitioner completes Module 3, **When** they want to integrate VLA capabilities, **Then** they can smoothly transition to Module 4
3. **Given** a practitioner needs simulation knowledge, **When** they reference Module 2, **Then** they can quickly locate Gazebo/Unity fundamentals

---

### Edge Cases

- How does the curriculum handle learners who want to focus only on simulation (Modules 2-3) without full VLA integration (Module 4)?
- What if a learner needs to complete the 13-week curriculum faster than 13 weeks?
- How do we support learners who get stuck on a specific week's hands-on project?

## Requirements

### Functional Requirements

- **FR-001**: Book structure MUST organize content into 4 distinct modules: Module 1 (ROS 2), Module 2 (Gazebo/Unity), Module 3 (NVIDIA Isaac), Module 4 (VLA)
- **FR-002**: Book MUST present a 13-week progressive learning path spanning all 4 modules
- **FR-003**: Each module MUST include clear learning objectives that build on previous modules
- **FR-003a**: Book MUST explicitly state prerequisites for Week 1: basic Python programming knowledge and command-line familiarity
- **FR-004**: Weeks 1-2 MUST cover Introduction to Physical AI including foundations, embodied intelligence, humanoid robotics landscape, and sensor systems (assumes basic Python/CLI knowledge)
- **FR-005**: Weeks 3-5 MUST cover ROS 2 Fundamentals including architecture, nodes/topics/services/actions, Python packages, and launch files
- **FR-006**: Weeks 6-7 MUST cover Robot Simulation with Gazebo including environment setup, URDF/SDF formats, physics simulation, and Unity introduction
- **FR-007**: Weeks 8-10 MUST cover NVIDIA Isaac Platform including Isaac SDK, Isaac Sim, AI-powered perception/manipulation, reinforcement learning, and sim-to-real transfer
- **FR-008**: Weeks 11-12 MUST cover Humanoid Robot Development including kinematics/dynamics, bipedal locomotion, manipulation/grasping, and human-robot interaction
- **FR-009**: Week 13 MUST cover Conversational Robotics including GPT integration, speech recognition, natural language understanding, and multi-modal interaction
- **FR-010**: Module 1 MUST detail ROS 2 nodes, topics, services, Python-to-ROS bridging via rclpy, and URDF for humanoids
- **FR-011**: Module 2 MUST detail Gazebo physics simulation (gravity, collisions), Unity rendering/HRI, and sensor simulation (LiDAR, depth cameras, IMUs)
- **FR-012**: Module 3 MUST detail NVIDIA Isaac Sim, Isaac ROS VSLAM, Nav2 path planning for bipedal movement
- **FR-012a**: Module 3 MUST provide cloud-based GPU alternatives (Google Colab, AWS SageMaker, Paperspace) with setup instructions for learners without local GPU access
- **FR-013**: Module 4 MUST detail voice-to-action with OpenAI Whisper, LLM-based cognitive planning, and natural language to ROS 2 action sequences
- **FR-014**: Book MUST include Autonomous Humanoid Capstone Project where a simulated robot receives voice commands, plans paths, navigates obstacles, identifies objects with computer vision, and manipulates them
- **FR-015**: Each week's content MUST include hands-on practical exercises that reinforce concepts
- **FR-015a**: Code examples MUST use inline MDX syntax-highlighted snippets for core concepts, with links to external GitHub repositories for complete working projects
- **FR-016**: Content structure MUST replace placeholder chapters with finalized curriculum content
- **FR-017**: Documentation directory structure MUST organize content by modules and weeks
- **FR-018**: Each module MUST clearly state prerequisites from previous modules
- **FR-019**: Capstone project MUST integrate concepts from all 4 modules
- **FR-020**: Content MUST progress from foundational concepts to advanced autonomous robotics systematically
- **FR-021**: All content files MUST include YAML frontmatter with title and sidebar_position fields
- **FR-022**: Book navigation MUST use autogenerated sidebars that automatically reflect docs/ folder structure
- **FR-022a**: Book MUST be built using Docusaurus v2/v3 as the static site generator platform
- **FR-022b**: Book MUST integrate Algolia DocSearch for instant search functionality with keyboard shortcuts and automatic content indexing
- **FR-023**: Module folders MUST be named module-XX-<name> (e.g., module-01-ros2, module-02-gazebo-unity) with index.md entry points
- **FR-024**: Week content files MUST use sidebar_position to maintain sequential ordering (weeks 1-13)

### Key Entities

- **Module**: A major thematic unit of the curriculum (4 total: ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA). Contains learning objectives, weekly content, and projects
- **Week**: A discrete time unit of the 13-week learning path. Contains topics, concepts, hands-on exercises, and references to relevant modules
- **Learning Objective**: Specific, measurable outcomes learners should achieve by completing a module or week
- **Hands-On Project**: Practical exercises integrated into weekly content, culminating in the capstone project
- **Capstone Project**: The Autonomous Humanoid - final integrative project demonstrating mastery across all modules
- **Concept**: Individual topics within weeks (e.g., "ROS 2 Nodes", "URDF", "Isaac Sim", "Voice-to-Action")
- **Prerequisite**: Knowledge or skills from earlier modules/weeks required for subsequent content

## Success Criteria

### Measurable Outcomes

- **SC-001**: Learners can identify and navigate to any of the 4 modules and 13 weeks within 30 seconds
- **SC-002**: 90% of learners completing Week 13 successfully integrate all 4 modules in the capstone project
- **SC-003**: Learners complete the sequential curriculum path from Week 1 to Week 13 without encountering missing or placeholder content
- **SC-004**: Educators can create a course syllabus using the book structure in under 2 hours
- **SC-005**: Learners can complete at least one hands-on exercise per week (13 total minimum)
- **SC-006**: 95% of technical references (ROS 2, Gazebo, Unity, NVIDIA Isaac, OpenAI Whisper) are accurate and current
- **SC-007**: Prerequisites for each module are explicitly stated and learners can assess readiness before starting
- **SC-008**: Capstone project requirements reference specific concepts from all 4 modules clearly
- **SC-009**: Content organization reduces learner confusion by 70% compared to placeholder chapter structure
- **SC-010**: 100% of promised curriculum topics (listed in modules/weeks) have corresponding content sections

## Clarifications

### Session 2025-12-04

- Q: What static site generator and documentation platform should be used to implement the autogenerated sidebars, module navigation, and responsive book structure? → A: Docusaurus v2/v3 - Purpose-built for technical docs with autogenerated sidebars, versioning, search, and MDX support
- Q: How should code examples, hands-on exercises, and technical implementations be delivered to learners? → A: Hybrid approach - Core snippets inline with syntax highlighting in MDX, full working projects linked to external GitHub repos
- Q: What happens when a learner doesn't have GPU access for NVIDIA Isaac exercises in Module 3? → A: Provide cloud-based alternatives with instructions for using platforms like Google Colab with GPU, AWS SageMaker, or Paperspace
- Q: How should learners search and discover specific topics across 13 weeks of content spanning 4 modules? → A: Algolia DocSearch - Docusaurus native integration with instant search, keyboard shortcuts, and automatic indexing
- Q: What foundational knowledge should learners have before starting Week 1 of the curriculum? → A: Basic programming literacy - Assumes basic Python knowledge and command-line familiarity; Weeks 1-2 focus on Physical AI concepts, not programming basics
