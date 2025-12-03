---
title: "Actuators and Sensors"
description: "Explore the hardware that enables robots to move and perceive - from motors and hydraulics to cameras and force sensors."
difficulty: "intermediate"
chapter: "Mechanics"
chapter_number: 2
sidebar_position: 3
learning_objectives:
  - "Understand different types of actuators and their trade-offs"
  - "Learn about various sensor modalities for robot perception"
  - "Distinguish between proprioceptive and exteroceptive sensing"
  - "Select appropriate actuators and sensors for applications"
prerequisites:
  - "Basic physics and electrical engineering"
  - "Chapter 1: Foundations"
estimated_time: "25 minutes"
keywords:
  - "actuators"
  - "motors"
  - "sensors"
  - "perception"
  - "proprioception"
  - "exteroception"
---

# Actuators and Sensors

## Introduction

**Actuators** convert electrical, hydraulic, or pneumatic energy into mechanical motion. **Sensors** measure the world and the robot's state. Together, they close the perception-action loop.

## Actuators: Making Robots Move

### Electric Motors

**DC Motors**
- **Brushed DC**: Simple, cheap, but require maintenance
- **Brushless DC (BLDC)**: Efficient, reliable, more expensive
- **Characteristics**: Good torque, speed control, compact

**Stepper Motors**
- Move in discrete steps (e.g., 1.8° per step)
- Open-loop control (no encoder needed)
- **Use cases**: 3D printers, CNC machines
- **Limitations**: Can lose steps under high load

**Servo Motors**
- Closed-loop control with encoder feedback
- Precise position and velocity control
- **Types**: Hobby servos (limited range), industrial servos (full rotation)

### Hydraulic Actuators

**Advantages**:
- Very high force/torque
- High power-to-weight ratio
- Compliant (soft) behavior

**Disadvantages**:
- Heavy (pump, reservoir, hoses)
- Maintenance (leaks, fluid changes)
- Noisy

**Applications**: Excavators, Boston Dynamics Atlas, heavy-duty robots

### Pneumatic Actuators

**Advantages**:
- Fast response
- Lightweight
- Naturally compliant (safe for human interaction)
- Cheap

**Disadvantages**:
- Limited force
- Poor position control (air compressibility)
- Requires compressed air source

**Applications**: Soft robots, grippers, industrial automation

### Series Elastic Actuators (SEA)

Spring between motor and load:
- **Benefits**: Force sensing, shock absorption, safety
- **Drawbacks**: Reduced bandwidth, added complexity
- **Use cases**: Legged robots, collaborative robots

### Comparison Table

| Actuator Type | Force/Torque | Speed | Precision | Compliance | Cost | Typical Use |
|---------------|--------------|-------|-----------|------------|------|-------------|
| **Brushless DC** | Medium | High | High | Low | Medium | Robot arms, drones |
| **Hydraulic** | Very High | Medium | Medium | Medium | High | Heavy machinery, legged robots |
| **Pneumatic** | Low-Medium | Very High | Low | High | Low | Grippers, soft robots |
| **SEA** | Medium | Medium | Medium | Very High | Medium | Cobots, exoskeletons |

## Sensors: Perceiving the World

Sensors divide into two categories:

**Proprioceptive**: Sense robot's own state (internal)
**Exteroceptive**: Sense environment (external)

### Proprioceptive Sensors

**Encoders**
- Measure joint position (angle or linear displacement)
- **Types**: Optical (quadrature), magnetic, absolute vs. incremental
- **Resolution**: 2048-4096 counts/revolution typical
- **Uses**: Joint position feedback, odometry

**Inertial Measurement Units (IMUs)**
- **Gyroscopes**: Measure angular velocity
- **Accelerometers**: Measure acceleration (including gravity)
- **Magnetometers**: Measure magnetic field (compass)
- **Sensor fusion**: Combine for orientation estimation
- **Drift**: Gyros drift over time, requires correction

**Force/Torque Sensors**
- Measure contact forces
- **Types**: Strain gauges, capacitive, optical
- **Uses**: Compliant control, assembly, grasping
- **Placement**: Wrist (6-axis F/T sensor), joints, fingertips

**Current Sensors**
- Measure motor current → estimate torque
- **Benefits**: Cheaper than F/T sensors
- **Limitations**: Friction, dynamics complicate torque estimation

### Exteroceptive Sensors

**Cameras**

*2D Cameras*
- **RGB**: Color images
- **Resolution**: 640×480 (VGA) to 4K+
- **Frame rate**: 30 FPS typical, 60-240 for fast motion
- **Trade-offs**: Resolution vs. speed vs. cost

*Depth Cameras*
- **Stereo**: Two cameras → triangulation (like human eyes)
- **Structured light**: Project pattern, measure distortion (Kinect v1)
- **Time-of-Flight (ToF)**: Measure light travel time (Kinect v2, RealSense)
- **Output**: RGBD image (color + depth per pixel)

**Lidar (Light Detection and Ranging)**
- Laser rangefinder, scans environment
- **2D Lidar**: Horizontal plane scan (e.g., Hokuyo, SICK)
- **3D Lidar**: Full 3D point cloud (e.g., Velodyne, Ouster)
- **Range**: 10-200m depending on model
- **Accuracy**: cm-level
- **Uses**: Autonomous vehicles, mapping, obstacle avoidance
- **Cost**: $100 (2D) to $10,000+ (automotive 3D)

**Radar**
- Radio waves for distance/velocity measurement
- **Advantages**: Works in fog, rain, dust (unlike cameras/lidar)
- **Disadvantages**: Lower resolution
- **Uses**: Automotive (adaptive cruise control), outdoor robots

**Ultrasonic Sensors**
- Sound waves for distance measurement
- **Range**: 2cm - 4m typical
- **Cheap**: $1-5 per sensor
- **Uses**: Parking sensors, indoor mobile robots

**Tactile Sensors**
- Detect contact, pressure, texture
- **Types**: Force-sensing resistors (FSR), capacitive, optical
- **Uses**: Grasping, manipulation, texture recognition

### Sensor Fusion

Combining multiple sensor modalities:

**Visual-Inertial Odometry (VIO)**
- Camera + IMU → robust pose estimation
- **Use**: Drones, VR headsets, AR devices

**Lidar-Camera Fusion**
- Lidar provides accurate 3D, camera provides semantics
- **Use**: Self-driving cars (perception pipeline)

**Multi-sensor SLAM**
- Combine cameras, lidar, wheel encoders, IMU
- **Benefits**: Robust to sensor failures, complementary strengths

## Sensor Characteristics

### Key Metrics

**Range**: Minimum and maximum measurable distance/value
**Resolution**: Smallest detectable change
**Accuracy**: How close to true value
**Precision**: Repeatability of measurement
**Bandwidth**: How fast sensor can update
**Field of View (FOV)**: Angular coverage (for vision/lidar sensors)

### Noise and Uncertainty

All sensors have noise:
- **Gaussian noise**: Random variations (can be filtered)
- **Systematic bias**: Consistent offset (requires calibration)
- **Outliers**: Occasional bad measurements (must be rejected)

**Filtering techniques**:
- Median filter (remove outliers)
- Low-pass filter (smooth noise)
- Kalman filter (optimal for Gaussian noise)
- Particle filter (non-Gaussian, multi-modal)

## Choosing Actuators and Sensors

### Design Considerations

**For Actuators**:
1. **Force/torque requirements**: Payload, acceleration
2. **Speed requirements**: Task duration, dynamic response
3. **Precision needed**: Positioning accuracy
4. **Environment**: Temperature, dust, moisture
5. **Safety**: Human proximity, compliance
6. **Cost and maintenance**: Budget, reliability

**For Sensors**:
1. **Task requirements**: What needs to be measured?
2. **Range and resolution**: Operating distance, detail needed
3. **Update rate**: Real-time vs. occasional measurement
4. **Environment**: Lighting, weather, obstacles
5. **Computational constraints**: Processing power, latency
6. **Redundancy**: Critical systems need backup sensors

### Example: Autonomous Car

**Actuators**:
- Electric motors (steering, drive)
- Hydraulic brakes

**Sensors**:
- Cameras (6+): 360° view
- Lidar: 3D environment mapping
- Radar: Long-range detection
- Wheel encoders: Odometry
- IMU: Orientation, acceleration
- GPS: Global position

## Summary

**Actuators** convert energy to motion:
- Electric motors: Versatile, precise
- Hydraulics: High force
- Pneumatics: Fast, compliant
- SEA: Safe, force-controlled

**Sensors** enable perception:
- Proprioceptive: Know your own state
- Exteroceptive: Sense the environment
- Fusion: Combine for robustness

Choosing the right actuators and sensors is critical for successful Physical AI systems.

---

**Next**: Continue to [Control Systems](./03-control.md) to learn how to make robots move precisely and safely.
