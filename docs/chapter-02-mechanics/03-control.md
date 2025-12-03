---
title: "Control Systems"
description: "Learn how to make robots move precisely and safely through feedback control, from PID to advanced model-based methods."
difficulty: "intermediate"
chapter: "Mechanics"
chapter_number: 2
sidebar_position: 4
learning_objectives:
  - "Understand feedback control and stability"
  - "Implement PID control for robotic systems"
  - "Apply model-based control techniques"
  - "Design safe controllers for human-robot interaction"
prerequisites:
  - "Kinematics and dynamics"
  - "Basic differential equations"
  - "Control theory (helpful but not required)"
estimated_time: "30 minutes"
keywords:
  - "PID control"
  - "feedback control"
  - "stability"
  - "model-based control"
  - "adaptive control"
  - "impedance control"
---

# Control Systems

## Introduction

**Control systems** make robots behave as desired. They convert high-level commands ("move to position X") into low-level actuator commands (voltages, currents) while accounting for disturbances, model uncertainty, and physical constraints.

## Feedback Control Fundamentals

### Open-Loop vs. Closed-Loop

**Open-Loop Control**:
```
Command → Controller → Actuator → Output
```
- No measurement of actual output
- Simple, fast
- **Problem**: Disturbances cause errors

**Closed-Loop (Feedback) Control**:
```
Command → [Controller] → Actuator → Output
    ↑                                  ↓
    └──────── Sensor ←─────────────────┘
```
- Measures output, corrects errors
- Robust to disturbances
- Can become unstable if poorly tuned

### Control Objectives

1. **Tracking**: Follow reference trajectory accurately
2. **Regulation**: Maintain desired state despite disturbances
3. **Stability**: System doesn't oscillate or diverge
4. **Performance**: Fast response, small overshoot
5. **Robustness**: Handle model errors and noise

## PID Control

**PID** (Proportional-Integral-Derivative) is the most common controller in robotics.

### The PID Equation

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de/dt

e(t) = r(t) - y(t)  # Error = reference - measurement
u(t): control output
```

**Three Terms**:

1. **Proportional (P)**: u = Kp · e
   - Larger error → stronger response
   - **Limitation**: Steady-state error

2. **Integral (I)**: u = Ki · ∫e dt
   - Eliminates steady-state error
   - **Danger**: Integrator windup (accumulate error during saturation)

3. **Derivative (D)**: u = Kd · de/dt
   - Anticipates future error (damping)
   - **Problem**: Amplifies noise

### PID Tuning

**Ziegler-Nichols Method**:
1. Set Ki=0, Kd=0
2. Increase Kp until system oscillates
3. Measure oscillation period T and critical Kp
4. Compute PID gains from formulas

**Manual Tuning**:
1. Start with Kp only
   - Increase until fast response but some overshoot
2. Add Kd to reduce overshoot
   - Too much → noise amplification
3. Add Ki to eliminate steady-state error
   - Too much → oscillations

**Modern Methods**:
- Auto-tuning algorithms
- Machine learning (learn optimal gains from data)

### Example: Joint Position Control

```python
class JointPIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.prev_error = 0

    def update(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        # PID output
        output = (self.Kp * error +
                  self.Ki * self.integral +
                  self.Kd * derivative)

        self.prev_error = error
        return output
```

### PID Limitations

- **Nonlinear systems**: PID is linear, robots are nonlinear
- **Coupled dynamics**: Multi-joint robots have interaction forces
- **Model uncertainty**: Fixed gains may not work everywhere
- **Constraints**: PID doesn't handle joint limits, obstacles

## Model-Based Control

Use robot dynamics model for better performance:

```
τ = M(q)·q̈_desired + C(q,q̇)·q̇ + G(q)
```

This **feedforward** term cancels dynamics, making PID's job easier.

### Computed Torque Control

```
τ = M(q)·[q̈_d + Kd·(q̇_d - q̇) + Kp·(q_d - q)] + C(q,q̇)·q̇ + G(q)

Feedforward: M·q̈_d + C·q̇ + G
Feedback: M·[Kd·ė + Kp·e]
```

**Benefits**:
- Linearizes system (simple to tune)
- Decouples joints
- High performance

**Requirements**:
- Accurate model (M, C, G)
- Fast computation
- Good state estimation

### Adaptive Control

Adapt model parameters online:

```
τ = M̂(q)·... + Ĉ·q̇ + Ĝ(q)
Update M̂, Ĉ, Ĝ based on tracking error
```

**Use cases**:
- Unknown payload
- Changing dynamics (grasping objects)
- Wear and tear

## Advanced Control Techniques

### Impedance Control

Control relationship between force and motion:

```
F = M_d·(ẍ - ẍ_d) + B_d·(ẋ - ẋ_d) + K_d·(x - x_d)

M_d: desired inertia
B_d: desired damping
K_d: desired stiffness
```

**Example**: Compliant grasping
- Soft (low Kd): Conform to object shape
- Stiff (high Kd): Precise positioning

**Applications**:
- Assembly tasks
- Human-robot collaboration
- Contact-rich manipulation

### Force Control

Regulate contact forces directly:

**Hybrid position/force control**:
- Position control in free-space directions
- Force control in contact directions

**Example**: Polishing
- Control position tangent to surface
- Control force normal to surface

### Learning-Based Control

Use machine learning to improve control:

**Reinforcement Learning**:
- Learn control policy from trial and error
- **Examples**: Quadruped locomotion, dexterous manipulation

**Imitation Learning**:
- Learn from human demonstrations
- **Examples**: Surgical robots, assembly tasks

**Neural Network Dynamics Models**:
- Learn M, C, G from data
- Useful when analytical model is hard

## Motion Planning and Control

Control operates at fast timescales (kHz), planning at slower timescales (Hz):

```
Planning Layer (slow):
  - Collision-free path
  - Trajectory optimization

Control Layer (fast):
  - Track planned trajectory
  - Reject disturbances
  - Ensure stability
```

**Trajectory Generation**:
- Minimum jerk trajectories (smooth)
- Quintic polynomials (position, velocity, acceleration)
- Spline interpolation

## Safety and Constraints

### Joint Limits

Prevent exceeding mechanical limits:
- Hard stops (damage)
- Soft limits (control system enforces)

**Methods**:
- Clamp commands
- Potential fields (repel from limits)
- Constrained optimization

### Collision Avoidance

**Static obstacles**:
- Pre-plan collision-free paths

**Dynamic obstacles**:
- Real-time replanning
- Model predictive control (MPC)

### Human Safety

**ISO 15066 (collaborative robots)**:
- Force/power limits
- Speed and separation monitoring
- Safety-rated monitored stop

**Safety features**:
- Force/torque limiting
- Collision detection
- E-stop circuits

## Practical Considerations

### Control Frequency

**Typical rates**:
- High-level planning: 10-100 Hz
- Position control: 100-1000 Hz
- Torque control: 1-10 kHz

Higher frequency → better performance but more computation.

### State Estimation

Control requires knowing current state:
- **Joint space**: Encoder measurements (direct)
- **Task space**: Forward kinematics or vision
- **Velocities**: Differentiate position (noisy) or use observers

**Kalman filtering**: Optimal state estimation with noise.

### Real-Time Constraints

Controllers must run reliably:
- **Hard real-time**: Missed deadline → danger
- **Soft real-time**: Occasional miss acceptable

**Solutions**:
- Real-time operating systems (RTOS)
- Priority scheduling
- Watchdog timers

## Summary

**Control systems** make robots move precisely:

**PID Control**:
- Simple, widely used
- Three gains to tune
- Limitations with nonlinearity

**Model-Based Control**:
- Uses robot dynamics
- Better performance
- Requires accurate model

**Advanced Techniques**:
- Impedance: Control force-motion relationship
- Force: Regulate contact forces
- Learning: Improve from data

**Safety**:
- Joint limits, collision avoidance
- Human-robot interaction standards

Effective control is essential for safe, precise, robust Physical AI systems.

---

**Congratulations!** You've completed Chapter 2: Mechanics and Control. You now understand how robots physically move and are controlled.

**What's Next**: More chapters coming soon covering perception, learning, and applications!
