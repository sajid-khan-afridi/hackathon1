---
title: "Week 6: Gazebo Fundamentals & Physics Simulation"
description: "Set up Gazebo simulation environments, work with URDF/SDF formats, and configure physics engines for humanoid robot testing."
sidebar_position: 2
week_id: "week-06-gazebo-fundamentals"
week_number: 6
module: "module-02-gazebo-unity"
learning_objectives:
  - "Install and configure Gazebo for robot simulation"
  - "Convert URDF models to SDF format for Gazebo"
  - "Configure physics engines (gravity, collisions, friction)"
  - "Spawn robots and objects in simulated environments"
prerequisites:
  - "Week 5: URDF Models"
  - "Module 1: ROS 2 Fundamentals complete"
estimated_time: "4 hours"
difficulty: "intermediate"
topics_covered:
  - "Gazebo architecture"
  - "SDF (Simulation Description Format)"
  - "Physics engines (ODE, Bullet, DART)"
  - "World files and environments"
  - "Gravity, friction, and collisions"
hands_on_exercises:
  - title: "Launch Humanoid Robot in Gazebo Environment"
    description: "Spawn a URDF humanoid model in Gazebo and test physics"
    estimated_time: "60 minutes"
    tools_required: ["ROS 2 Humble", "Gazebo Fortress", "URDF from Week 5"]
keywords:
  - "Gazebo"
  - "simulation"
  - "SDF"
  - "physics"
  - "collisions"
  - "gravity"
references:
  - title: "Gazebo Tutorials"
    url: "https://gazebosim.org/docs"
  - title: "Integrating Gazebo with ROS 2"
    url: "https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html"
---

# Week 6: Gazebo Fundamentals & Physics Simulation

**Gazebo** is an open-source robot simulator that provides realistic physics, sensor simulation, and 3D visualization. This week you'll set up Gazebo environments to test humanoid robots safely before deploying to hardware.

## What is Gazebo?

Gazebo simulates:
- **Physics**: Gravity, friction, collisions, contact dynamics
- **Sensors**: Cameras, LiDAR, IMUs, force/torque sensors
- **Actuators**: Motors with accurate dynamics
- **Environments**: Indoor/outdoor worlds with obstacles

**Key Advantage**: Test dangerous or expensive scenarios (falling, collisions) in simulation first.

## Gazebo Architecture

```
Gazebo Server (gzserver)
  ├─ Physics Engine (ODE/Bullet/DART)
  ├─ World State (robot poses, object positions)
  └─ Sensor Simulation

Gazebo Client (gzclient)
  └─ 3D Visualization

ROS 2 Bridge (ros_gz_bridge)
  └─ Connect Gazebo topics ↔ ROS 2 topics
```

## SDF: Simulation Description Format

While URDF describes robot structure, **SDF (Simulation Description Format)** is Gazebo's native format with additional features:
- Multiple robots per file
- Plugin support
- More physics properties
- Light sources, cameras

### URDF to SDF Conversion

Gazebo automatically converts URDF to SDF, but you can manually convert:

```bash
gz sdf -p robot.urdf > robot.sdf
```

### SDF World File Example

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="humanoid_world">

    <!-- Physics Engine Config -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Gravity -->
    <gravity>0 0 -9.81</gravity>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Light Source -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
    </light>

  </world>
</sdf>
```

## Physics Engine Configuration

Gazebo supports multiple physics engines:

| Engine | Pros | Cons | Use Case |
|--------|------|------|----------|
| **ODE** (default) | Fast, stable | Less accurate for complex contacts | General robotics |
| **Bullet** | Good collision detection | Slower | Manipulation, grasping |
| **DART** | Accurate dynamics | Computationally expensive | Research, biomechanics |

### Tuning Physics Parameters

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
  <real_time_factor>1.0</real_time_factor>  <!-- Run at real-time speed -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz -->

  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>  <!-- More iterations = more accuracy -->
    </solver>
  </ode>
</physics>
```

## Spawning Robots in Gazebo

### Method 1: Launch File (ROS 2 + Gazebo)

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # Spawn Robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'humanoid_robot',
            '-file', '/path/to/robot.urdf',
            '-x', '0', '-y', '0', '-z', '0.5'
        ]
    )

    return LaunchDescription([gazebo, spawn_robot])
```

### Method 2: Command Line

```bash
# Launch Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# Spawn robot (in another terminal)
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file robot.urdf -x 0 -y 0 -z 1.0
```

## Collision and Contact Dynamics

### Contact Properties

```xml
<collision name="foot_collision">
  <geometry>
    <box size="0.2 0.1 0.05"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>  <!-- Friction coefficient -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <bounce>
      <restitution_coefficient>0.0</restitution_coefficient>  <!-- 0 = no bounce -->
    </bounce>
  </surface>
</collision>
```

**High Friction** (µ = 1.0): Good for walking (prevents foot slip)
**Low Friction** (µ = 0.1): Ice-like surface for testing balance

## Hands-On Exercise

### Exercise: Launch Humanoid Robot in Gazebo

**Objective**: Spawn your Week 5 URDF humanoid in Gazebo and test physics.

**Steps**:

1. **Install Gazebo**:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

2. **Create Launch File** (`spawn_humanoid.launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_pkg'),
        'urdf', 'simple_humanoid.urdf'
    )

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'humanoid',
                '-file', urdf_file,
                '-z', '1.0'  # Spawn 1m above ground
            ]
        )
    ])
```

3. **Launch Gazebo + Robot**:
```bash
# Terminal 1: Start Gazebo
ros2 launch gazebo_ros gazebo.launch.py

# Terminal 2: Spawn humanoid
ros2 launch my_robot_pkg spawn_humanoid.launch.py
```

4. **Observe Physics**:
   - Robot should fall and collide with ground
   - Check if joints behave correctly
   - Apply forces (Gazebo GUI → right-click robot → "Apply Force")

5. **Experiment**:
   - Change spawn height (`-z 2.0`)
   - Add friction to feet in URDF
   - Modify gravity in world file

**Expected Outcome**: Humanoid spawns in Gazebo, falls, and collides with ground plane with realistic physics.

**Complete Code**: [GitHub: gazebo-humanoid-spawn](https://github.com/gazebosim/ros_gz_project_template)

## Summary

- **Gazebo** simulates physics, sensors, and environments for robot testing
- **SDF** is Gazebo's format, more powerful than URDF for simulation
- **Physics engines** (ODE, Bullet, DART) compute dynamics, collisions, and contact forces
- **World files** define environments, lighting, and global parameters
- **Spawning robots** requires launch files or CLI commands

## Next Steps

Continue to [Week 7: Unity & Sensor Simulation](./week-07-unity-sensors.md) to explore photorealistic rendering and advanced sensor simulation with Unity.

---

**Estimated Time**: 4 hours
**Difficulty**: Intermediate
