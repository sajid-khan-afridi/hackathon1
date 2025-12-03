---
title: "Kinematics and Dynamics"
description: "Master the mathematics of robot motion - from position and orientation to forces and torques."
difficulty: "intermediate"
chapter: "Mechanics"
chapter_number: 2
sidebar_position: 2
learning_objectives:
  - "Understand forward and inverse kinematics"
  - "Compute velocities and accelerations using Jacobians"
  - "Apply Newton-Euler dynamics to robot systems"
  - "Analyze workspace and singularities"
prerequisites:
  - "Linear algebra (vectors, matrices)"
  - "Basic calculus (derivatives)"
  - "Chapter 1: Foundations"
estimated_time: "30 minutes"
keywords:
  - "forward kinematics"
  - "inverse kinematics"
  - "Jacobian"
  - "dynamics"
  - "workspace"
  - "singularities"
---

# Kinematics and Dynamics

## Introduction

**Kinematics** describes motion without considering forces - where a robot is and how fast it's moving. **Dynamics** includes forces and torques that cause motion. Together, they form the mathematical foundation of robot motion.

## Forward Kinematics

**Forward kinematics** answers: "Given joint angles, where is the end effector?"

### Example: 2-Link Planar Arm

Consider a simple 2-link robot arm in a plane:

```
Link 1: length L₁, angle θ₁ (from horizontal)
Link 2: length L₂, angle θ₂ (from Link 1)

End effector position (x, y):
x = L₁ cos(θ₁) + L₂ cos(θ₁ + θ₂)
y = L₁ sin(θ₁) + L₂ sin(θ₁ + θ₂)
```

**Properties**:
- **Unique solution**: Given joint angles → one end effector position
- **Fast to compute**: Direct calculation
- **Exact**: No approximations needed

### Homogeneous Transformations

For 3D robots, we use 4×4 transformation matrices:

```
T = [R  p]
    [0  1]

R: 3×3 rotation matrix
p: 3×1 position vector
```

Forward kinematics chains transformations:
```
T_end = T₀₁ × T₁₂ × T₂₃ × ... × Tₙ_end
```

Common representations:
- **Denavit-Hartenberg (DH) parameters**: Standard for serial manipulators
- **Euler angles**: Roll, pitch, yaw
- **Quaternions**: Avoid gimbal lock, smooth interpolation

## Inverse Kinematics (IK)

**Inverse kinematics** answers: "What joint angles produce a desired end effector pose?"

### Challenges

IK is harder than forward kinematics:

1. **Multiple solutions**: Same position may have different joint configurations
2. **No solution**: Position outside workspace
3. **Infinite solutions**: Redundant robots (7+ DOF)
4. **Numerical complexity**: Often requires iterative solvers

### Example: 2-Link Arm IK

Given desired position (x, y), compute (θ₁, θ₂):

```python
# Elbow up solution
c₂ = (x² + y² - L₁² - L₂²) / (2·L₁·L₂)
θ₂ = atan2(±√(1 - c₂²), c₂)  # ± gives two solutions
θ₁ = atan2(y, x) - atan2(L₂·sin(θ₂), L₁ + L₂·cos(θ₂))
```

**Two solutions**: "Elbow up" and "elbow down"

### IK Solution Methods

**Analytical (Closed-Form)**
- Exact, fast
- Limited to specific geometries (6-DOF with spherical wrist)
- Example: PUMA arm, KUKA iiwa

**Numerical (Iterative)**
- General purpose, works for any robot
- Approximate, may not converge
- Methods: Jacobian pseudoinverse, optimization, machine learning

**Common approach**:
```python
while error > tolerance:
    Δq = J⁺ · Δx  # Jacobian pseudoinverse
    q = q + α · Δq  # Update joint angles
```

## Velocity Kinematics: The Jacobian

The **Jacobian matrix** J relates joint velocities to end effector velocity:

```
v = J(q) · q̇

v: end effector velocity (6×1 for 3D: linear + angular)
q̇: joint velocities (n×1)
J: Jacobian matrix (6×n)
```

### Why Jacobians Matter

1. **Velocity control**: Control end effector speed directly
2. **Inverse kinematics**: Iterative IK uses Jacobian
3. **Singularity detection**: det(J) = 0 indicates singularity
4. **Force mapping**: Torques related to forces via Jᵀ

### Example: 2-Link Jacobian

```
J = [-L₁·sin(θ₁) - L₂·sin(θ₁+θ₂),  -L₂·sin(θ₁+θ₂)]
    [ L₁·cos(θ₁) + L₂·cos(θ₁+θ₂),   L₂·cos(θ₁+θ₂)]
```

## Singularities

A **singularity** occurs when the Jacobian loses rank - the robot loses one or more degrees of freedom.

### Types of Singularities

**Boundary singularities**: Arm fully extended or retracted
**Interior singularities**: Wrist flips, aligned joint axes

**Consequences**:
- Cannot move in certain directions
- Requires infinite joint velocity
- Loss of controllability

**Example**: Human arm fully extended - can't extend further, only move sideways.

## Workspace Analysis

The **workspace** is all positions the end effector can reach.

**Reachable workspace**: Any point reachable (maybe only from one direction)
**Dexterous workspace**: Points reachable with any orientation

### Factors Affecting Workspace

- Joint limits (mechanical stops, software limits)
- Link lengths and configurations
- Obstacles and collisions
- Singularities

## Robot Dynamics

Dynamics describes how forces and torques cause motion:

```
τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)

τ: joint torques (n×1)
M(q): inertia matrix (n×n)
C(q,q̇): Coriolis and centrifugal terms
G(q): gravity vector
```

### Newton-Euler Formulation

Recursive algorithm for efficient computation:
1. **Forward pass**: Compute velocities and accelerations
2. **Backward pass**: Compute forces and torques

**Complexity**: O(n) vs. O(n⁴) for naive approach

### Applications of Dynamics

1. **Simulation**: Predict robot motion given torques
2. **Control**: Compute required torques for desired motion
3. **Design**: Size motors, estimate energy consumption
4. **Safety**: Predict collision forces

## Summary

**Kinematics** provides the geometric foundation:
- Forward kinematics: joints → position (unique, fast)
- Inverse kinematics: position → joints (multiple/no solutions, harder)
- Jacobian: relates velocities, essential for control
- Singularities: configurations to avoid

**Dynamics** adds physics:
- Equation of motion relates forces/torques to acceleration
- Essential for accurate control and simulation
- Newton-Euler provides efficient computation

---

**Next**: Continue to [Actuators and Sensors](./02-actuators-sensors.md) to learn how robots physically move and perceive.
