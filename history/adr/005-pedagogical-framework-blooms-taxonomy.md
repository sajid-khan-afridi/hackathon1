# ADR-005: Pedagogical Framework - Bloom's Taxonomy Progression

> **Scope**: Defines the learning progression model for 13-week Physical AI curriculum using Bloom's Taxonomy-based skill development.

- **Status:** Accepted
- **Date:** 2025-12-04
- **Feature:** 001-curriculum-restructure
- **Context:** Curriculum requires pedagogically sound progression from foundational concepts to advanced autonomous robotics, ensuring learners build skills systematically across 13 weeks

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: YES - Affects how all curriculum content is structured, sequenced, and assessed across all modules
     2) Alternatives: YES - Problem-based learning, spiral curriculum, competency-based models all considered
     3) Scope: YES - Cross-cutting concern affecting content authoring, learning objectives, hands-on exercises, and assessment criteria
-->

## Decision

We will adopt **Bloom's Taxonomy-Based Progression** as the pedagogical framework for the 13-week curriculum:

### Bloom's Taxonomy Learning Levels (Applied to Curriculum):

**Weeks 1-2: Remember & Understand** (Foundational Knowledge)
- **Cognitive Level**: Recall facts, explain concepts
- **Content**: Introduction to Physical AI, embodied intelligence theory, humanoid robotics landscape
- **Learning Activities**: Reading, concept diagrams, vocabulary building
- **Assessment**: Can define Physical AI, explain differences from traditional AI, identify key components

**Weeks 3-5: Apply** (Practical Skills)
- **Cognitive Level**: Execute procedures, implement solutions
- **Content**: ROS 2 hands-on exercises (node creation, topic publishing, URDF modeling)
- **Learning Activities**: Coding exercises, configuration tasks, system setup
- **Assessment**: Can create ROS 2 nodes, publish/subscribe to topics, define robot models

**Weeks 6-7: Analyze** (Comparative Understanding)
- **Cognitive Level**: Compare approaches, break down systems
- **Content**: Simulation environments (Gazebo vs Unity trade-offs), sensor modeling techniques
- **Learning Activities**: Environment comparisons, system architecture analysis
- **Assessment**: Can compare simulation tools, analyze sensor accuracy, troubleshoot integration issues

**Weeks 8-10: Evaluate** (Critical Decision-Making)
- **Cognitive Level**: Assess trade-offs, critique solutions
- **Content**: AI-powered perception, reinforcement learning strategies, sim-to-real transfer challenges
- **Learning Activities**: Algorithm selection, performance evaluation, design critiques
- **Assessment**: Can evaluate perception algorithms, justify RL choices, assess sim-to-real gap mitigation

**Weeks 11-13: Create** (Synthesis & Innovation)
- **Cognitive Level**: Design systems, integrate components
- **Content**: Humanoid kinematics, bipedal locomotion, conversational robotics, Autonomous Humanoid Capstone
- **Learning Activities**: Full-stack integration, capstone project development, original problem-solving
- **Assessment**: Can design and implement autonomous humanoid robot integrating voice, vision, navigation, manipulation

### Progression Enforcement Mechanisms:

1. **Prerequisites Field**: Each week explicitly lists prior weeks required (enforces sequential learning)
   ```yaml
   prerequisites:
     - "Week 1: Introduction to Physical AI"
     - "Week 2: Embodied Intelligence"
   ```

2. **Difficulty Progression**: Metadata tracks cognitive complexity
   - Weeks 1-5: `difficulty: "beginner"`
   - Weeks 6-7: `difficulty: "intermediate"`
   - Weeks 8-10: `difficulty: "intermediate"` → `"advanced"`
   - Weeks 11-13: `difficulty: "advanced"`

3. **Hands-On Exercise Complexity**: Minimum 1 exercise per week, increasing in scope
   - Weeks 1-2: 15-30 minute exploratory exercises
   - Weeks 3-5: 45-60 minute implementation exercises
   - Weeks 6-7: 1-2 hour integration exercises
   - Weeks 8-10: 2-3 hour evaluation/optimization exercises
   - Weeks 11-13: Multi-week capstone project (cumulative)

4. **Learning Objectives Taxonomy**: Written using Bloom's action verbs
   - Remember: "Identify", "List", "Define"
   - Understand: "Explain", "Summarize", "Describe"
   - Apply: "Implement", "Execute", "Configure"
   - Analyze: "Compare", "Diagnose", "Differentiate"
   - Evaluate: "Assess", "Critique", "Justify"
   - Create: "Design", "Integrate", "Develop"

### Capstone Integration (Week 13):
- **Synthesis Requirement**: Autonomous Humanoid project integrates ALL Bloom's levels
  - Remember: Recall ROS 2 architecture (Module 1)
  - Understand: Explain sensor fusion (Module 2)
  - Apply: Implement perception pipeline (Module 3)
  - Analyze: Compare locomotion strategies (Module 4)
  - Evaluate: Justify design decisions (all modules)
  - Create: Build complete autonomous system (all modules)

## Consequences

### Positive

- **Pedagogically Proven**: Bloom's Taxonomy is research-backed framework with 60+ years of validation in education
- **Clear Progression**: Learners understand skill development trajectory (foundational → advanced → mastery)
- **Measurable Outcomes**: Taxonomy provides action verbs for concrete learning objectives ("implement" > "understand concepts")
- **Prevents Skill Gaps**: Sequential prerequisite enforcement ensures learners don't skip foundational knowledge
- **Supports Diverse Learners**: Explicit progression accommodates different paces (strong learners advance quickly, struggling learners get clear scaffolding)
- **Facilitates Assessment**: Each Bloom's level has associated assessment strategies (quizzes for Remember, projects for Create)
- **Content Authoring Guidance**: Writers know which cognitive level each week targets, reducing ambiguity
- **Capstone Justification**: "Create" level at Week 13 naturally integrates all prior weeks, aligning with FR-019 (capstone requirement)

### Negative

- **Rigid Sequencing**: Bloom's progression doesn't allow non-linear learning (can't skip to advanced topics without foundational weeks)
  - **Mitigation**: Acceptable trade-off for structured bootcamp format; experienced learners can self-assess prerequisites
- **Cognitive Level Ambiguity**: Some content spans multiple Bloom's levels (e.g., ROS 2 involves both Apply and Analyze)
  - **Mitigation**: Assign primary Bloom's level per week; acknowledge secondary levels in learning objectives
- **Assessment Overhead**: Each Bloom's level requires different assessment types (tests, exercises, projects) increasing evaluation complexity
  - **Mitigation**: Focus on hands-on exercises (practical assessment) over traditional exams; capstone serves as final evaluation
- **Taxonomy Limitations**: Bloom's is one of many pedagogical models; may not capture all learning dimensions (e.g., metacognition, affective domain)
  - **Mitigation**: Bloom's Cognitive Domain sufficient for technical skills curriculum; affective/metacognitive learning captured in capstone reflection
- **Content Density Imbalance**: "Create" level requires more time than "Remember" - Week 13 may need more hours than Week 1
  - **Mitigation**: `estimated_time` frontmatter field varies by week; clearly communicate time expectations upfront

### Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Learners skip prerequisites and struggle in advanced weeks | Medium | High | Explicit prerequisite checks, recommended self-assessment quizzes before each module |
| Bloom's progression too slow for experienced learners | Medium | Medium | Provide "fast-track" guidance for skipping weeks where prerequisites met; optional "deep dive" content for enrichment |
| "Create" level capstone too ambitious for 13-week timeline | Low | High | Capstone scoped to integration (not novel research); provide starter code and templates to reduce implementation burden |
| Cognitive level mismatch between weeks (e.g., Week 7 harder than Week 8) | Low | Medium | Peer review curriculum before launch; iterate based on learner feedback; adjust difficulty ratings |

## Alternatives Considered

### Alternative 1: Problem-Based Learning (PBL)

**Description**: Start with complex problem (Autonomous Humanoid) on Day 1, learn concepts as needed to solve it

**Pros**:
- High motivation (immediate real-world relevance)
- Learner-driven knowledge acquisition
- Develops problem-solving skills early

**Cons**:
- ❌ Too advanced for beginners without foundational knowledge (research.md finding: learners need 2-3 weeks of basics)
- ❌ Cognitive overload risk (trying to learn ROS 2, Isaac, and VLA simultaneously)
- ❌ Harder to structure as sequential curriculum (non-linear learning paths)
- ❌ Requires expert facilitation (less suited for self-paced online book)

**Rejected because**: Inappropriate for beginner audience; 13-week bootcamp format better suits structured progression

### Alternative 2: Spiral Curriculum

**Description**: Revisit core topics (sensors, actuators, control) in each module at increasing depth

**Pros**:
- Reinforcement through repetition
- Flexibility for late joiners (can enter at any module)
- Accommodates forgetting curve

**Cons**:
- ❌ Content redundancy (same topics covered multiple times)
- ❌ Context-switching overhead (ROS 2 → Gazebo → ROS 2 → Isaac → ROS 2 pattern)
- ❌ Learners may perceive curriculum as repetitive or slow
- ❌ Harder to write (avoiding exact duplication while increasing depth)

**Rejected because**: Linear progressive model better suits 13-week intensive format (research.md Bloom's recommendation)

### Alternative 3: Competency-Based Progression

**Description**: Learners progress by demonstrating mastery of competencies (not time-based weeks)

**Pros**:
- Self-paced (fast learners advance quickly, slow learners get more time)
- Mastery-focused (must prove competence before advancing)
- Personalized learning paths

**Cons**:
- ❌ Incompatible with "13-week curriculum" spec requirement (FR-001: explicit week structure)
- ❌ Harder to implement in static book format (requires adaptive assessment system)
- ❌ Coordination challenges for cohort-based learning (students at different points)
- ❌ More complex content authoring (need to define competency rubrics, not just learning objectives)

**Rejected because**: Violates spec requirement for 13-week time-based structure

### Alternative 4: Constructivist Learning (Unstructured Exploration)

**Description**: Provide resources and tools, let learners explore Physical AI concepts freely without prescribed sequence

**Pros**:
- Maximum learner autonomy
- Supports intrinsic motivation
- Discovery-based learning

**Cons**:
- ❌ No guaranteed learning progression (learners may miss critical foundations)
- ❌ Incompatible with sequential week structure (FR-001 requirement)
- ❌ Harder to assess outcomes (no common curriculum)
- ❌ High dropout risk (lack of structure overwhelms beginners)

**Rejected because**: Violates spec requirement for structured 13-week curriculum; inappropriate for bootcamp format

## References

- Feature Spec: [specs/001-curriculum-restructure/spec.md](../../specs/001-curriculum-restructure/spec.md) - FR-001 (13-week structure), FR-015 (hands-on exercises)
- Implementation Plan: [specs/001-curriculum-restructure/plan.md](../../specs/001-curriculum-restructure/plan.md)
- Research: [specs/001-curriculum-restructure/research.md](../../specs/001-curriculum-restructure/research.md) - Section 1 (Bloom's Taxonomy decision)
- Data Model: [specs/001-curriculum-restructure/data-model.md](../../specs/001-curriculum-restructure/data-model.md) - Week entity definitions
- Related ADRs: ADR-004 (Module-Based Organization)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) - Principle I (Content-First, progressive complexity)
- Academic Source: Bloom, B.S. (1956). Taxonomy of Educational Objectives, Handbook I: The Cognitive Domain
- Academic Source: Freeman et al. (2014). Active learning increases student performance in science, engineering, and mathematics. PNAS
