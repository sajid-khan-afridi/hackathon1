---
title: "Key Concepts in Physical AI"
description: "Master the essential terminology, principles, and concepts that form the foundation of Physical AI and robotics."
difficulty: "beginner"
chapter: "Foundations"
chapter_number: 1
sidebar_position: 4
learning_objectives:
  - "Understand fundamental terminology in Physical AI"
  - "Recognize different types of robotic systems"
  - "Comprehend key principles and frameworks"
  - "Apply concepts to analyze real-world robots"
prerequisites:
  - "Introduction to Physical AI"
  - "History and Evolution of Robotics"
estimated_time: "20 minutes"
keywords:
  - "robotics terminology"
  - "degrees of freedom"
  - "kinematics"
  - "control systems"
  - "autonomy levels"
---

# Key Concepts in Physical AI

## Core Terminology

### Degrees of Freedom (DOF)

**Degrees of Freedom** specify the number of independent parameters that define a robot's configuration.

- **1-DOF**: Single axis of movement (e.g., elevator)
- **3-DOF**: XYZ positioning (e.g., 3D printer head)
- **6-DOF**: Full 3D position + orientation (e.g., industrial robot arm)
- **7+ DOF**: Redundant robots with extra flexibility

**Example**: A human arm has 7 DOF:
- 3 DOF in shoulder
- 1 DOF in elbow
- 3 DOF in wrist

### End Effector

The **end effector** is the device at the end of a robotic arm designed to interact with the environment:

- **Grippers**: Parallel jaw, suction cups, soft grippers
- **Tools**: Welding torches, spray guns, drills
- **Sensors**: Cameras, force sensors for inspection

### Workspace

The **workspace** is the volume of space a robot can reach:

- **Reachable workspace**: All points the end effector can reach
- **Dexterous workspace**: Points reachable with any orientation
- **Obstacle-free workspace**: Accounting for environmental constraints

## Types of Robotic Systems

### By Mobility

**Fixed-Base Robots**
- Industrial robot arms
- Collaborative robots (cobots)
- Surgical robots

**Mobile Robots**
- **Wheeled**: Cars, delivery robots, warehouse bots
- **Legged**: Humanoids, quadrupeds (Spot, ANYmal)
- **Flying**: Drones, quadcopters
- **Aquatic**: Underwater vehicles, surface vessels

**Hybrid Systems**
- Mobile manipulators (arm on mobile base)
- Drones with grippers
- Humanoid robots with locomotion and manipulation

### By Application

| Type | Purpose | Examples |
|------|---------|----------|
| **Industrial** | Manufacturing, assembly | KUKA, ABB, Fanuc robots |
| **Service** | Assistance, delivery | Roomba, delivery robots |
| **Medical** | Surgery, rehabilitation | da Vinci surgical system |
| **Agricultural** | Harvesting, monitoring | Fruit-picking robots |
| **Exploration** | Space, underwater | Mars rovers, ROVs |
| **Military** | Reconnaissance, EOD | PackBot, bomb disposal |

## Levels of Autonomy

Robots operate at different levels of autonomy:

**Level 0: Full Manual Control**
- Human operates every movement
- Example: Teleoperated surgical robots

**Level 1: Assisted Control**
- Robot provides stability or force feedback
- Example: Power steering in cars

**Level 2: Partial Automation**
- Robot handles specific tasks, human monitors
- Example: Lane-keeping assistance

**Level 3: Conditional Automation**
- Robot operates independently in defined scenarios
- Example: Highway autopilot

**Level 4: High Automation**
- Fully autonomous in specific domains
- Example: Warehouse robots

**Level 5: Full Automation**
- Operates anywhere, anytime without human intervention
- Example: Future general-purpose robots (not yet achieved)

## Perception-Action Cycle

The fundamental loop of Physical AI:

```
Environment
    ↓
 [Sensors] ──> Raw data (images, lidar, IMU)
    ↓
 [Perception] ──> Semantic understanding (objects, obstacles)
    ↓
 [Planning] ──> Decide what to do
    ↓
 [Control] ──> How to execute actions
    ↓
 [Actuators] ──> Physical movement
    ↓
Environment (modified)
```

Each component has challenges:
- **Sensors**: Noise, calibration, failure modes
- **Perception**: Ambiguity, occlusion, lighting
- **Planning**: Complexity, real-time constraints
- **Control**: Stability, precision, disturbances
- **Actuators**: Bandwidth, force limits, wear

## Key Principles

### Embodied Intelligence

Intelligence emerges from the interaction between brain, body, and environment:
- The body shapes what can be learned
- Physical constraints simplify control
- Morphological computation offloads processing

### Subsumption Architecture

Rodney Brooks' influential framework:
- Behavior-based control (not deliberative planning)
- Layers of competence build on each other
- Fast, reactive behaviors at low levels
- Higher levels modulate lower levels

### Sense-Plan-Act vs. Reactive Control

**Traditional Sense-Plan-Act**:
1. Build complete world model
2. Plan optimal action sequence
3. Execute plan
- **Pros**: Optimal solutions
- **Cons**: Slow, brittle to changes

**Reactive/Behavior-Based**:
1. Direct sensor-to-actuator mappings
2. Simple behaviors combine for complex results
3. No explicit world model
- **Pros**: Fast, robust
- **Cons**: Limited to local decisions

**Modern Approach**: Hybrid architectures
- Reactive layer for safety and real-time response
- Deliberative layer for planning and learning

## Modeling and Simulation

Physical AI development relies heavily on simulation:

### Forward Kinematics
Given joint angles → compute end effector position

### Inverse Kinematics
Given desired position → compute required joint angles
- Often multiple solutions
- Optimization for smoothness and joint limits

### Dynamics Simulation
Predict how forces and torques affect motion:
- Physics engines (PyBullet, MuJoCo, Isaac Sim)
- Essential for learning and testing
- Sim-to-real gap remains challenging

## Common Frameworks and Tools

**Robot Operating System (ROS)**
- Standard middleware for robotics
- Message passing between components
- Large ecosystem of packages

**Gazebo / Isaac Sim**
- Physics simulation environments
- Sensor simulation
- Integration with ROS

**MoveIt**
- Motion planning framework
- Collision avoidance
- Inverse kinematics solvers

**PyBullet / MuJoCo**
- Physics engines for reinforcement learning
- Fast, accurate dynamics
- Widely used in research

## Summary

Understanding these key concepts provides the vocabulary and mental models for Physical AI:

- **Terminology**: DOF, end effectors, workspace
- **System types**: Fixed vs. mobile, application domains
- **Autonomy levels**: From teleoperation to full automation
- **Perception-action cycle**: The core loop of Physical AI
- **Key principles**: Embodiment, reactive control, hybrid architectures
- **Tools**: ROS, simulators, planning frameworks

---

**Congratulations!** You've completed Chapter 1: Foundations. You now have the essential knowledge to understand Physical AI systems.

**Next**: Continue to [Chapter 2: Mechanics and Control](../chapter-02-mechanics/index.md) to dive deeper into how robots move and are controlled.
