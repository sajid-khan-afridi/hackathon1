---
title: "Week 5: Python Packages & URDF for Humanoid Robots"
description: "Structure ROS 2 projects with ament_python and define humanoid robot models using URDF for simulation and control."
sidebar_position: 6
week_id: "week-05-ros2-python-urdf"
week_number: 5
module: "module-01-ros2"
learning_objectives:
  - "Organize ROS 2 Python packages with proper structure"
  - "Define robot models using URDF (Unified Robot Description Format)"
  - "Specify joints, links, and kinematic chains for humanoid robots"
  - "Visualize URDF models in RViz"
prerequisites:
  - "Week 4: ROS 2 Nodes & Topics"
  - "Basic XML syntax"
estimated_time: "4 hours"
difficulty: "intermediate"
topics_covered:
  - "ROS 2 package structure (ament_python)"
  - "URDF syntax and elements"
  - "Joints (revolute, prismatic, fixed)"
  - "Links (visual, collision, inertial)"
  - "Kinematic chains for humanoids"
  - "RViz visualization"
hands_on_exercises:
  - title: "Define a Humanoid Robot URDF Model"
    description: "Create a URDF file for a simplified humanoid with arms and legs"
    estimated_time: "60 minutes"
    tools_required: ["ROS 2 Humble", "RViz", "Text editor"]
keywords:
  - "URDF"
  - "robot model"
  - "joints"
  - "links"
  - "kinematics"
  - "RViz"
  - "humanoid"
references:
  - title: "URDF Tutorials"
    url: "https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html"
  - title: "URDF XML Specification"
    url: "http://wiki.ros.org/urdf/XML"
---

# Week 5: Python Packages & URDF for Humanoid Robots

This week focuses on structuring ROS 2 projects and defining robot models using URDF—the standard format for describing robot geometry, kinematics, and dynamics.

## ROS 2 Package Structure

### ament_python Package Layout

```
my_robot_pkg/
├── package.xml          # Package metadata
├── setup.py             # Python package configuration
├── setup.cfg            # Install rules
├── my_robot_pkg/        # Python module
│   ├── __init__.py
│   ├── node1.py         # Node implementations
│   └── node2.py
├── launch/              # Launch files (optional)
│   └── robot_launch.py
├── urdf/                # Robot models (optional)
│   └── robot.urdf
└── config/              # Configuration files (optional)
    └── params.yaml
```

### package.xml Example

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.1.0</version>
  <description>Humanoid robot control package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## URDF: Unified Robot Description Format

**URDF** is an XML format for describing:
- **Links**: Rigid bodies (torso, arms, legs, head)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Visual geometry**: 3D meshes or primitive shapes for rendering
- **Collision geometry**: Simplified shapes for physics simulation
- **Inertial properties**: Mass, inertia matrix for dynamics

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Left Upper Arm Link -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

### Link Elements

**Visual**: Appearance in simulation/RViz
```xml
<visual>
  <geometry>
    <box size="0.3 0.2 0.6"/>  <!-- or cylinder, sphere, mesh -->
  </geometry>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
</visual>
```

**Collision**: Simplified shape for physics
```xml
<collision>
  <geometry>
    <box size="0.3 0.2 0.6"/>
  </geometry>
</collision>
```

**Inertial**: Mass and inertia matrix
```xml
<inertial>
  <mass value="5.0"/>
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
</inertial>
```

### Joint Types

| Type | Description | Use Case |
|------|-------------|----------|
| **revolute** | Rotating joint with limits | Elbow, knee, shoulder |
| **continuous** | Rotating joint without limits | Wheels |
| **prismatic** | Sliding joint | Telescoping arm |
| **fixed** | Rigid connection | Sensor mount |
| **floating** | 6-DOF (position + orientation) | Base of mobile robot |
| **planar** | Motion in 2D plane | -- |

### Humanoid Kinematic Chain Example

```
torso (base)
 ├─ left_shoulder (revolute) → left_upper_arm
 │   └─ left_elbow (revolute) → left_forearm
 │       └─ left_wrist (revolute) → left_hand
 ├─ right_shoulder (revolute) → right_upper_arm
 │   └─ right_elbow (revolute) → right_forearm
 │       └─ right_wrist (revolute) → right_hand
 ├─ neck (revolute) → head
 ├─ left_hip (revolute) → left_thigh
 │   └─ left_knee (revolute) → left_shin
 │       └─ left_ankle (revolute) → left_foot
 └─ right_hip (revolute) → right_thigh
     └─ right_knee (revolute) → right_shin
         └─ right_ankle (revolute) → right_foot
```

**Total DOF**: 2 shoulders + 2 elbows + 2 wrists + 1 neck + 2 hips + 2 knees + 2 ankles = **13+ joints**

## Visualizing URDF in RViz

```bash
# Install RViz and joint_state_publisher
sudo apt install ros-humble-rviz2 ros-humble-joint-state-publisher-gui

# Launch URDF visualization
ros2 launch urdf_tutorial display.launch.py model:=/path/to/robot.urdf
```

**GUI**: Use sliders to move joints and verify kinematic structure.

## Hands-On Exercise

### Exercise: Define a Humanoid Robot URDF Model

**Objective**: Create a simplified humanoid URDF with torso, arms, and legs.

**Steps**:

1. **Create URDF File** (`simple_humanoid.urdf`):
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry><box size="0.3 0.2 0.6"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material>
    </visual>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_arm"/>
    <origin xyz="0.15 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_arm">
    <visual>
      <geometry><cylinder radius="0.05" length="0.4"/></geometry>
      <origin xyz="0 0 -0.2"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Arm (mirror of left) -->
  <joint name="right_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="right_arm"/>
    <origin xyz="-0.15 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_arm">
    <visual>
      <geometry><cylinder radius="0.05" length="0.4"/></geometry>
      <origin xyz="0 0 -0.2"/>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip" type="revolute">
    <parent link="torso"/>
    <child link="left_leg"/>
    <origin xyz="0.1 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="150" velocity="1.0"/>
  </joint>

  <link name="left_leg">
    <visual>
      <geometry><cylinder radius="0.06" length="0.5"/></geometry>
      <origin xyz="0 0 -0.25"/>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.06" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Leg (mirror of left) -->
  <joint name="right_hip" type="revolute">
    <parent link="torso"/>
    <child link="right_leg"/>
    <origin xyz="-0.1 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="150" velocity="1.0"/>
  </joint>

  <link name="right_leg">
    <visual>
      <geometry><cylinder radius="0.06" length="0.5"/></geometry>
      <origin xyz="0 0 -0.25"/>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.06" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.001"/>
    </inertial>
  </link>

</robot>
```

2. **Visualize in RViz**:
```bash
ros2 launch urdf_tutorial display.launch.py model:=simple_humanoid.urdf
```

3. **Experiment**:
   - Add elbow joints to arms
   - Add knee joints to legs
   - Adjust joint limits and observe ranges

**Expected Outcome**: A 3D humanoid robot visualized in RViz with movable joints.

**Complete Code**: [GitHub: urdf-humanoid-examples](https://github.com/ros/urdf_tutorial)

## Summary

- **ROS 2 packages** organize code with ament_python structure
- **URDF** defines robot geometry, kinematics, and dynamics
- **Links** represent rigid bodies with visual, collision, and inertial properties
- **Joints** connect links (revolute, prismatic, fixed)
- **Humanoid models** require 10-40+ joints for arms, legs, torso, and head
- **RViz** visualizes URDF models with interactive joint control

## Next Steps

Continue to [Module 2: Robot Simulation](../module-02-gazebo-unity/) to bring your URDF models to life in Gazebo and Unity simulation environments.

---

**Estimated Time**: 4 hours
**Difficulty**: Intermediate
