---
title: "Week 11: Humanoid Kinematics & Dynamics"
description: "Calculate forward and inverse kinematics for humanoid manipulators, understand dynamics equations, and implement whole-body control."
sidebar_position: 2
week_id: "week-11-humanoid-kinematics"
week_number: 11
module: "module-04-vla-conversational"
learning_objectives:
  - "Calculate forward kinematics for humanoid arms"
  - "Solve inverse kinematics for target poses"
  - "Understand Jacobian matrices for velocity control"
  - "Apply dynamics equations (Lagrangian, Newton-Euler)"
prerequisites:
  - "Modules 1-3 complete"
  - "Linear algebra basics (matrices, vectors)"
estimated_time: "5 hours"
difficulty: "advanced"
topics_covered:
  - "Forward kinematics"
  - "Inverse kinematics"
  - "Jacobian matrices"
  - "Dynamics (Lagrangian, Newton-Euler)"
  - "Whole-body control"
hands_on_exercises:
  - title: "Calculate Forward Kinematics for Humanoid Arm"
    description: "Implement FK using DH parameters in Python"
    estimated_time: "60 minutes"
    tools_required: ["Python", "NumPy", "Matplotlib"]
keywords:
  - "kinematics"
  - "dynamics"
  - "Jacobian"
  - "inverse kinematics"
references:
  - title: "Introduction to Robotics: Mechanics and Control (Craig)"
    url: "https://www.pearson.com/en-us/subject-catalog/p/introduction-to-robotics-mechanics-and-control/P200000003481"
---

# Week 11: Humanoid Kinematics & Dynamics

Master the mathematical foundations of humanoid motion: kinematics (position/velocity) and dynamics (forces/torques). This week covers forward/inverse kinematics, Jacobians, and whole-body control.

## Forward Kinematics

**Forward Kinematics (FK)** computes end-effector pose from joint angles.

### Denavit-Hartenberg (DH) Parameters

Standard notation for defining kinematic chains:

| Joint | θ (rotation) | d (offset) | a (link length) | α (twist) |
|-------|-------------|-----------|----------------|-----------|
| 1 (shoulder) | θ₁ | 0 | 0 | 90° |
| 2 (elbow) | θ₂ | 0 | L₁ | 0° |
| 3 (wrist) | θ₃ | 0 | L₂ | 0° |

### Transformation Matrix

```python
import numpy as np

def dh_matrix(theta, d, a, alpha):
    """Compute DH transformation matrix."""
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                 np.cos(alpha),                d],
        [0,              0,                             0,                            1]
    ])

def forward_kinematics(joint_angles, dh_params):
    """Compute end-effector pose from joint angles."""
    T = np.eye(4)  # Identity matrix
    for i, (theta, d, a, alpha) in enumerate(dh_params):
        theta_total = theta + joint_angles[i]
        T = T @ dh_matrix(theta_total, d, a, alpha)
    return T

# Example: 3-DOF arm
dh_params = [
    (0, 0, 0, np.pi/2),      # Shoulder
    (0, 0, 0.3, 0),          # Elbow (L1 = 0.3m)
    (0, 0, 0.25, 0)          # Wrist (L2 = 0.25m)
]
joint_angles = [np.pi/4, np.pi/3, 0]
T = forward_kinematics(joint_angles, dh_params)
print("End-effector position:", T[:3, 3])
```

## Inverse Kinematics

**Inverse Kinematics (IK)** solves for joint angles given desired end-effector pose.

### Analytical IK (2-DOF Planar Arm)

```python
def inverse_kinematics_2dof(x, y, L1, L2):
    """Analytical IK for 2-link planar arm."""
    # Law of cosines for elbow angle
    c2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = np.arccos(c2)

    # Shoulder angle
    k1 = L1 + L2 * np.cos(theta2)
    k2 = L2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1, theta2

# Example
x_target, y_target = 0.4, 0.3
theta1, theta2 = inverse_kinematics_2dof(x_target, y_target, L1=0.3, L2=0.25)
print(f"Joint angles: θ1={np.degrees(theta1):.1f}°, θ2={np.degrees(theta2):.1f}°")
```

### Numerical IK (Jacobian Pseudo-Inverse)

For complex humanoids, use iterative methods:

```python
def jacobian_ik(target_pose, current_angles, dh_params, max_iter=100, tol=0.01):
    """Iterative IK using Jacobian pseudo-inverse."""
    angles = current_angles.copy()

    for _ in range(max_iter):
        # Current pose
        T = forward_kinematics(angles, dh_params)
        current_pose = T[:3, 3]

        # Error
        error = target_pose - current_pose
        if np.linalg.norm(error) < tol:
            return angles  # Converged

        # Jacobian (numerical approximation)
        J = numerical_jacobian(angles, dh_params)

        # Update angles: Δθ = J⁺ * error
        delta_angles = np.linalg.pinv(J) @ error
        angles += 0.1 * delta_angles  # Step size = 0.1

    return angles  # Max iterations reached
```

## Jacobian Matrix

The **Jacobian** relates joint velocities to end-effector velocities:

**v = J(θ) · θ̇**

Where:
- **v**: End-effector linear/angular velocity (6×1)
- **J(θ)**: Jacobian matrix (6×n)
- **θ̇**: Joint velocities (n×1)

### Applications

1. **Velocity Control**: Directly command end-effector velocity
2. **Singularity Detection**: det(J) = 0 → loss of DOF
3. **Force Control**: τ = J^T · F (joint torques from end-effector force)

## Dynamics

**Dynamics** computes forces/torques needed to achieve desired motion.

### Equation of Motion

**τ = M(θ)θ̈ + C(θ, θ̇)θ̇ + G(θ)**

Where:
- **M(θ)**: Inertia matrix (configuration-dependent)
- **C(θ, θ̇)**: Coriolis and centrifugal forces
- **G(θ)**: Gravity forces

### Computed Torque Control

```python
def computed_torque_control(theta, theta_dot, theta_desired, theta_dot_desired):
    """Computed torque controller for trajectory tracking."""
    # PD gains
    Kp, Kd = 100, 20

    # Desired acceleration (PD control)
    theta_ddot_desired = (Kp * (theta_desired - theta) +
                          Kd * (theta_dot_desired - theta_dot))

    # Compute dynamics terms
    M = inertia_matrix(theta)
    C = coriolis_matrix(theta, theta_dot)
    G = gravity_vector(theta)

    # Control law
    tau = M @ theta_ddot_desired + C @ theta_dot + G
    return tau
```

## Hands-On Exercise

### Exercise: Calculate Forward Kinematics for Humanoid Arm

**Objective**: Implement FK for 3-DOF humanoid arm.

**Steps**:

1. **Define DH Parameters**:
```python
# Shoulder-Elbow-Wrist arm
dh_params = [
    (0, 0, 0, np.pi/2),      # Shoulder pitch
    (0, 0, 0.35, 0),         # Elbow (upper arm = 35cm)
    (0, 0, 0.30, 0)          # Wrist (forearm = 30cm)
]
```

2. **Implement FK Function** (use code from above)

3. **Test Multiple Configurations**:
```python
# Straight arm
angles_straight = [0, 0, 0]
T = forward_kinematics(angles_straight, dh_params)
print("Straight:", T[:3, 3])  # Should be [0, 0, 0.65]

# Bent arm
angles_bent = [0, np.pi/2, 0]
T = forward_kinematics(angles_bent, dh_params)
print("Bent:", T[:3, 3])  # Should be [0.35, 0, 0.30]
```

4. **Visualize**:
```python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Plot arm configuration
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Compute joint positions
positions = [np.array([0, 0, 0])]  # Base
T = np.eye(4)
for theta, d, a, alpha in dh_params:
    T = T @ dh_matrix(theta + joint_angles[i], d, a, alpha)
    positions.append(T[:3, 3])

# Plot links
for i in range(len(positions)-1):
    ax.plot([positions[i][0], positions[i+1][0]],
            [positions[i][1], positions[i+1][1]],
            [positions[i][2], positions[i+1][2]], 'o-')

plt.show()
```

**Expected Outcome**: Arm visualized in 3D, end-effector position matches FK calculation.

**Complete Code**: [GitHub: Robotics Kinematics Examples](https://github.com/petercorke/robotics-toolbox-python)

## Summary

- **Forward kinematics** computes end-effector pose from joint angles (DH parameters)
- **Inverse kinematics** solves for joint angles to reach target pose (analytical or numerical)
- **Jacobian** relates joint velocities to end-effector velocities
- **Dynamics** models forces/torques required for motion (M, C, G terms)
- **Whole-body control** coordinates multiple limbs simultaneously

## Next Steps

Continue to [Week 12: Bipedal Locomotion & Manipulation](./week-12-bipedal-locomotion.md) to apply kinematics for walking and grasping.

---

**Estimated Time**: 5 hours
**Difficulty**: Advanced
