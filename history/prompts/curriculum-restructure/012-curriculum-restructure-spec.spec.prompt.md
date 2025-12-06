---
id: 012
title: Physical AI Curriculum Restructure Specification
stage: spec
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: curriculum-restructure
branch: 001-curriculum-restructure
user: Sajid Khan Afridi
command: /sp.specify
labels: ["curriculum", "book-structure", "physical-ai", "learning-path", "specification"]
links:
  spec: specs/001-curriculum-restructure/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-curriculum-restructure/spec.md
  - specs/001-curriculum-restructure/checklists/requirements.md
tests:
  - Specification quality validation checklist (all items passed)
---

## Prompt

Restructure book content to match finalized 4-module curriculum: Module 1 (ROS 2), Module 2 (Gazebo/Unity), Module 3 (NVIDIA Isaac), Module 4 (VLA).
Replace placeholder chapters with 13-week progressive learning path including hands-on projects and autonomous humanoid capstone.

This will create:
- spec.md: Detailed requirements for restructuring docs/ directory
- plan.md: Architecture for organizing modules/weeks/assessments
- tasks.md: Step-by-step implementation tasks

Module 1: The Robotic Nervous System (ROS 2)
Focus: Middleware for robot control.
- ROS 2 Nodes, Topics, and Services.
- Bridging Python Agents to ROS controllers using rclpy.
- Understanding URDF (Unified Robot Description Format) for humanoids.

Module 2: The Digital Twin (Gazebo & Unity)
Focus: Physics simulation and environment building.
- Simulating physics, gravity, and collisions in Gazebo.
- High-fidelity rendering and human-robot interaction in Unity.
- Simulating sensors: LiDAR, Depth Cameras, and IMUs.

Module 3: The AI-Robot Brain (NVIDIA Isaac™)
Focus: Advanced perception and training.
- NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation.
- Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation.
- Nav2: Path planning for bipedal humanoid movement.

Module 4: Vision-Language-Action (VLA)
Focus: The convergence of LLMs and Robotics.
- Voice-to-Action: Using OpenAI Whisper for voice commands.
- Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions.
- Capstone Project: The Autonomous Humanoid. A final project where a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it.

Weekly Breakdown:
- Weeks 1-2: Introduction to Physical AI (foundations, embodied intelligence, humanoid robotics, sensor systems)
- Weeks 3-5: ROS 2 Fundamentals (architecture, nodes/topics/services/actions, Python packages, launch files)
- Weeks 6-7: Robot Simulation with Gazebo (environment setup, URDF/SDF, physics, Unity visualization)
- Weeks 8-10: NVIDIA Isaac Platform (Isaac SDK/Sim, AI perception/manipulation, reinforcement learning, sim-to-real)
- Weeks 11-12: Humanoid Robot Development (kinematics/dynamics, bipedal locomotion, manipulation, HRI)
- Week 13: Conversational Robotics (GPT integration, speech recognition, NLU, multi-modal interaction)

## Response snapshot

Successfully created comprehensive specification for Physical AI book curriculum restructure:

**Branch**: 001-curriculum-restructure
**Spec File**: specs/001-curriculum-restructure/spec.md

### Specification Contents

**3 Prioritized User Stories**:
- P1: Learner following progressive 13-week curriculum (core value)
- P2: Educator planning course using book structure (multiplier)
- P3: Self-directed learner accessing module reference (flexibility)

**20 Functional Requirements** covering:
- 4-module structure (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- 13-week progressive learning path with specific weekly topics
- Module-specific content details (nodes, topics, services, simulation, perception, VLA)
- Autonomous Humanoid capstone project integrating all modules
- Hands-on exercises, prerequisites, and content progression

**10 Success Criteria** including:
- Navigation efficiency (30 seconds to any module/week)
- Completion rates (90% capstone success)
- Educator efficiency (2-hour syllabus creation)
- Technical accuracy (95% reference accuracy)
- Content completeness (100% promised topics covered)

**Quality Validation**: All checklist items passed
- No implementation details in spec
- Technology-agnostic success criteria
- Testable requirements with Given/When/Then scenarios
- Clear scope boundaries and dependencies
- Edge cases identified (GPU access, flexible learning paths, timing, support)

### Key Design Decisions

1. **Three User Personas**: Balanced sequential learners, educators, and self-directed practitioners
2. **Progressive Structure**: 13-week path ensures systematic skill building from foundations to advanced
3. **Measurable Outcomes**: All success criteria include specific metrics (percentages, timeframes, counts)
4. **Module Integration**: Capstone project explicitly integrates all 4 modules demonstrating complete learning
5. **Flexible Access**: While structured as 13-week path, supports jumping to specific modules with clear prerequisites

## Outcome

- ✅ Impact: Complete specification for restructuring Physical AI book content into 4-module, 13-week curriculum with autonomous humanoid capstone
- 🧪 Tests: Specification quality checklist validates all requirements are testable, measurable, and unambiguous
- 📁 Files: Created spec.md (115 lines, 20 FRs, 10 SCs) and requirements.md checklist (all items passed)
- 🔁 Next prompts: Ready for /sp.plan to design documentation architecture and organization strategy
- 🧠 Reflection: Specification successfully avoids implementation details while maintaining concrete, verifiable requirements. Technology names (ROS 2, NVIDIA Isaac) are appropriately included as subject matter, not implementation choices.

## Evaluation notes (flywheel)

- Failure modes observed: None. Initial script execution had minor parameter syntax issues but branch and spec were created successfully.
- Graders run and results (PASS/FAIL): Specification Quality Checklist - ALL PASS (Content Quality 4/4, Requirement Completeness 8/8, Feature Readiness 4/4)
- Prompt variant (if applicable): Standard /sp.specify workflow with comprehensive user input
- Next experiment (smallest change to try): Proceed with /sp.plan to validate that specification provides sufficient detail for architectural planning
