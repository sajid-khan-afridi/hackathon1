# Data Model: Physical AI Curriculum Structure

**Feature**: 001-curriculum-restructure
**Date**: 2025-12-04
**Purpose**: Define curriculum entities, relationships, and validation rules for the 13-week learning path

## Overview

This data model captures the structure of the Physical AI book curriculum, organized as a hierarchy of Modules → Weeks → Learning Content. All entities are represented as Markdown files with YAML frontmatter, following Docusaurus conventions.

## Entity Definitions

### 1. Module

**Description**: A major thematic unit of the curriculum (4 total: ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA). Groups related weeks and defines module-level learning objectives.

**File Location**: `docs/module-XX-<name>/index.md`

**Attributes**:
- `module_id` (string, unique): Identifier (e.g., "module-01-ros2")
- `module_number` (integer, 1-4): Sequential module number
- `module_name` (string, required): Display name (e.g., "ROS 2 Fundamentals")
- `description` (string, required): Brief overview for SEO and metadata
- `sidebar_position` (integer, required): Ordering in navigation (1-4)
- `weeks_included` (array[integer], required): List of weeks covered (e.g., [1, 2, 3, 4, 5])
- `learning_objectives` (array[string], required): Module-level objectives (3-5 items)
- `prerequisites` (array[string], optional): Required knowledge from prior modules
- `estimated_time` (string, required): Total time estimate (e.g., "3 weeks")
- `difficulty` (enum, required): "beginner" | "intermediate" | "advanced"
- `keywords` (array[string], required): Key concepts for search indexing

**Relationships**:
- HAS MANY: Week (1 module contains 2-5 weeks)
- FOLLOWS: Module (sequential ordering, Module 2 follows Module 1)

**Validation Rules**:
- `module_number` must be unique (1, 2, 3, 4)
- `weeks_included` must be contiguous and within 1-13 range
- `prerequisites` can only reference earlier module_ids

**Example**:
```yaml
---
title: "Module 1: ROS 2 Fundamentals"
description: "Master Robot Operating System 2 (ROS 2) architecture, nodes, topics, services, and Python integration for humanoid robotics."
sidebar_position: 1
module_id: "module-01-ros2"
module_number: 1
weeks_included: [1, 2, 3, 4, 5]
learning_objectives:
  - "Understand ROS 2 architecture and communication patterns"
  - "Create and manage ROS 2 nodes, topics, and services"
  - "Implement Python-based ROS 2 packages using rclpy"
  - "Define robot models using URDF for humanoid systems"
prerequisites:
  - "Basic Python programming (functions, classes, modules)"
  - "Command-line proficiency (navigation, file operations)"
estimated_time: "5 weeks"
difficulty: "beginner"
keywords:
  - "ROS 2"
  - "nodes"
  - "topics"
  - "services"
  - "rclpy"
  - "URDF"
---
```

---

### 2. Week

**Description**: A discrete time unit of the 13-week learning path. Contains topics, concepts, hands-on exercises, and references to relevant modules.

**File Location**: `docs/module-XX-<name>/week-YY-<topic>.md`

**Attributes**:
- `week_id` (string, unique): Identifier (e.g., "week-03-ros2-architecture")
- `week_number` (integer, 1-13, unique): Sequential week number
- `week_title` (string, required): Display name (e.g., "Week 3: ROS 2 Architecture")
- `description` (string, required): Brief summary for SEO and metadata
- `sidebar_position` (integer, required): Ordering within module (2, 3, 4...)
- `module` (string, required): Parent module_id (e.g., "module-01-ros2")
- `learning_objectives` (array[string], required): Week-specific objectives (3-5 items)
- `prerequisites` (array[string], required): Required knowledge from earlier weeks
- `estimated_time` (string, required): Reading/exercise time (e.g., "2 hours")
- `difficulty` (enum, required): "beginner" | "intermediate" | "advanced"
- `topics_covered` (array[string], required): Main topics (3-7 items)
- `hands_on_exercises` (array[Exercise], required): Practical activities (min 1 per week)
- `keywords` (array[string], required): Key concepts for search indexing
- `references` (array[Reference], optional): External resources (ROS 2 docs, papers)

**Relationships**:
- BELONGS TO: Module (each week belongs to exactly 1 module)
- FOLLOWS: Week (sequential ordering, Week 4 follows Week 3)
- HAS MANY: Concept (1 week contains 3-7 concepts)
- HAS MANY: Exercise (1 week contains 1+ hands-on exercises)

**Validation Rules**:
- `week_number` must be unique (1-13)
- `module` must reference valid module_id
- `sidebar_position` must be unique within module
- `prerequisites` can only reference earlier week_numbers
- `hands_on_exercises` must have at least 1 item (FR-015)

**Example**:
```yaml
---
title: "Week 3: ROS 2 Architecture"
description: "Explore ROS 2 architecture, DDS middleware, and the publish-subscribe communication pattern for distributed robotic systems."
sidebar_position: 3
week_id: "week-03-ros2-architecture"
week_number: 3
module: "module-01-ros2"
learning_objectives:
  - "Explain ROS 2 architecture and DDS middleware layer"
  - "Distinguish between ROS 1 and ROS 2 design principles"
  - "Identify key components: nodes, topics, services, actions"
prerequisites:
  - "Week 1: Introduction to Physical AI"
  - "Week 2: Embodied Intelligence"
estimated_time: "2 hours"
difficulty: "beginner"
topics_covered:
  - "ROS 2 architecture overview"
  - "DDS middleware and QoS policies"
  - "Publish-subscribe pattern"
  - "Nodes, topics, services, actions"
hands_on_exercises:
  - title: "Inspect ROS 2 nodes and topics"
    description: "Use CLI tools to explore running ROS 2 nodes"
    estimated_time: "30 minutes"
    tools_required: ["ROS 2 Humble", "Ubuntu 22.04 or Docker"]
keywords:
  - "ROS 2 architecture"
  - "DDS"
  - "middleware"
  - "nodes"
  - "topics"
references:
  - title: "ROS 2 Design Documentation"
    url: "https://design.ros2.org/"
---
```

---

### 3. Exercise (Embedded in Week)

**Description**: Hands-on practical activity within a week. Minimum 1 per week (FR-015).

**Attributes**:
- `title` (string, required): Exercise name
- `description` (string, required): What learners will do
- `estimated_time` (string, required): Completion time estimate
- `tools_required` (array[string], required): Software/hardware dependencies
- `difficulty` (enum, optional): "easy" | "medium" | "hard"
- `github_repo` (string, optional): Link to complete working project

**Validation Rules**:
- Each Week must have at least 1 Exercise (FR-015)
- `estimated_time` should be realistic (15 min - 2 hours)

**Example**:
```yaml
hands_on_exercises:
  - title: "Create a minimal ROS 2 publisher node"
    description: "Write a Python node that publishes String messages to a topic"
    estimated_time: "45 minutes"
    tools_required: ["ROS 2 Humble", "Python 3.10+", "rclpy"]
    difficulty: "easy"
    github_repo: "https://github.com/example/ros2-minimal-publisher"
```

---

### 4. Concept (Implicit in Week Content)

**Description**: Individual topics within weeks (e.g., "ROS 2 Nodes", "URDF", "Isaac Sim"). Represented as Markdown sections, not separate frontmatter.

**Attributes** (Markdown structure, not YAML):
- Section heading (##, ###)
- Explanatory text
- Inline code snippets (15-30 lines, per research.md)
- Diagrams/images (where applicable)
- Cross-references to earlier concepts

**Example**:
```markdown
## ROS 2 Nodes

A **node** is an executable process in a ROS 2 system. Each node is responsible for a specific task (e.g., sensor reading, motor control).

\```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started')
\```

See [Week 2: ROS 2 Installation](./week-02-ros2-installation.md) for setup instructions.
```

---

### 5. Capstone Project

**Description**: The Autonomous Humanoid - final integrative project demonstrating mastery across all modules. Covered in Week 13 and dedicated capstone/ folder.

**File Location**: `docs/capstone/`

**Attributes**:
- `capstone_id` (string, unique): "autonomous-humanoid"
- `title` (string, required): "Autonomous Humanoid Capstone Project"
- `description` (string, required): Project overview
- `modules_integrated` (array[string], required): All 4 module_ids
- `weeks_integrated` (array[integer], required): All weeks 1-13
- `requirements` (array[Requirement], required): Voice, vision, navigation, manipulation (FR-014)
- `evaluation_criteria` (array[string], required): Success metrics

**Relationships**:
- INTEGRATES: Module (all 4 modules)
- REFERENCES: Week (concepts from weeks 1-13)

**Validation Rules**:
- Must reference all 4 modules (FR-019)
- Requirements must align with FR-014 (voice commands, path planning, object identification, manipulation)

**Example**:
```yaml
---
title: "Autonomous Humanoid Capstone Project"
description: "Build a simulated humanoid robot that receives voice commands, plans paths, navigates obstacles, identifies objects, and manipulates them."
sidebar_position: 1
capstone_id: "autonomous-humanoid"
modules_integrated:
  - "module-01-ros2"
  - "module-02-gazebo-unity"
  - "module-03-nvidia-isaac"
  - "module-04-vla-conversational"
weeks_integrated: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
requirements:
  - component: "Voice Command Interface"
    description: "Use OpenAI Whisper for speech recognition"
    source_module: "module-04-vla-conversational"
  - component: "Path Planning"
    description: "Implement Nav2 for bipedal navigation"
    source_module: "module-03-nvidia-isaac"
  - component: "Object Detection"
    description: "Computer vision for object identification"
    source_module: "module-03-nvidia-isaac"
  - component: "Manipulation"
    description: "Grasp and move objects using ROS 2 MoveIt"
    source_module: "module-01-ros2"
evaluation_criteria:
  - "Robot successfully interprets voice commands"
  - "Path planning avoids obstacles in simulation"
  - "Object detection accuracy >80%"
  - "Successful object manipulation (grasp + move)"
---
```

---

## Entity Relationships Diagram

```
[Module 1: ROS 2]
    ├── Week 1: Intro Physical AI
    ├── Week 2: Embodied Intelligence
    ├── Week 3: ROS 2 Architecture
    │   ├── Concept: Nodes
    │   ├── Concept: Topics
    │   └── Exercise: Inspect nodes
    ├── Week 4: Nodes & Topics
    └── Week 5: Python & URDF

[Module 2: Gazebo/Unity]
    ├── Week 6: Gazebo Fundamentals
    └── Week 7: Unity & Sensors

[Module 3: NVIDIA Isaac]
    ├── Week 8: Isaac SDK/Sim
    ├── Week 9: AI Perception
    └── Week 10: RL & Sim-to-Real

[Module 4: VLA Conversational]
    ├── Week 11: Humanoid Kinematics
    ├── Week 12: Bipedal Locomotion
    └── Week 13: Conversational Robotics

[Capstone: Autonomous Humanoid]
    ← integrates all modules
    ← references all weeks
```

## State Transitions

### Module Progression
- **Initial State**: Learner starts Module 1
- **Transition**: Completes all weeks in module → unlocks next module
- **Final State**: Completes Module 4 → eligible for Capstone

### Week Progression
- **Initial State**: Learner opens Week N
- **Transition**: Completes hands-on exercises → marks week complete
- **Final State**: All weeks complete → ready for Capstone

## Validation Checklist

Before implementation (tasks.md phase):
- [ ] All 4 modules defined with valid frontmatter
- [ ] All 13 weeks mapped to correct modules
- [ ] Each week has ≥1 hands-on exercise (FR-015)
- [ ] Capstone references all 4 modules (FR-019)
- [ ] Prerequisites only reference earlier weeks/modules
- [ ] sidebar_position values are unique within each module
- [ ] All module/week_ids are unique across curriculum

---

**References**:
- Feature Spec: specs/001-curriculum-restructure/spec.md (Key Entities section)
- Research: specs/001-curriculum-restructure/research.md (Folder structure, frontmatter patterns)
- Constitution: .specify/memory/constitution.md (Principle II: Docusaurus Architecture)
