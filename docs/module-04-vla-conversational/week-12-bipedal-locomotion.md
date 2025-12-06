---
title: "Week 12: Bipedal Locomotion & Manipulation"
description: "Implement walking controllers, balance algorithms, and grasping strategies using MoveIt for humanoid robots."
sidebar_position: 3
week_id: "week-12-bipedal-locomotion"
week_number: 12
module: "module-04-vla-conversational"
learning_objectives:
  - "Implement zero-moment point (ZMP) walking controllers"
  - "Develop balance recovery algorithms"
  - "Use MoveIt for manipulation planning"
  - "Execute pick-and-place tasks with humanoid arms"
prerequisites:
  - "Week 11: Humanoid Kinematics"
estimated_time: "6 hours"
difficulty: "advanced"
topics_covered:
  - "Bipedal locomotion"
  - "ZMP (Zero-Moment Point)"
  - "Balance control"
  - "MoveIt motion planning"
  - "Grasping strategies"
hands_on_exercises:
  - title: "Implement Grasping Controller with MoveIt"
    description: "Plan and execute pick-and-place with humanoid arm"
    estimated_time: "90 minutes"
    tools_required: ["ROS 2 Humble", "MoveIt 2", "Isaac Sim or Gazebo"]
keywords:
  - "bipedal locomotion"
  - "walking"
  - "balance"
  - "ZMP"
  - "MoveIt"
  - "grasping"
references:
  - title: "MoveIt 2 Documentation"
    url: "https://moveit.picknik.ai/humble/index.html"
---

# Week 12: Bipedal Locomotion & Manipulation

Implement walking controllers for stable bipedal locomotion and use MoveIt for manipulation planning. This week combines dynamics, kinematics, and motion planning.

## Bipedal Locomotion Fundamentals

**Challenge**: Humans have unstable equilibrium—falling is the default state. Walking requires continuous balance recovery.

### Gait Cycle

1. **Single Support**: One foot on ground, other leg swinging
2. **Double Support**: Both feet on ground (transition phase)
3. **Swing Phase**: Foot lifts and moves forward
4. **Stance Phase**: Foot contacts ground and bears weight

**Typical Gait**:
- 60% single support
- 20% double support (each transition)
- 20% swing phase

## Zero-Moment Point (ZMP)

**ZMP**: Point on the ground where the sum of moments from gravity and inertia is zero.

**Stability Condition**: ZMP must remain inside the support polygon (convex hull of contact points).

### ZMP Calculation

```python
import numpy as np

def compute_zmp(com_position, com_acceleration, foot_positions, mass=70.0):
    """
    Compute ZMP given center of mass (CoM) state.

    Args:
        com_position: [x, y, z] of CoM
        com_acceleration: [ax, ay, az] of CoM
        foot_positions: List of foot contact points [[x1,y1], [x2,y2], ...]
        mass: Robot mass (kg)

    Returns:
        ZMP position [x, y] on ground plane
    """
    g = 9.81  # Gravity

    # ZMP formula (Vukobratovic)
    zmp_x = com_position[0] - (com_position[2] / (com_acceleration[2] + g)) * com_acceleration[0]
    zmp_y = com_position[1] - (com_position[2] / (com_acceleration[2] + g)) * com_acceleration[1]

    return np.array([zmp_x, zmp_y])

# Example
com_pos = [0.0, 0.0, 0.9]  # CoM at 90cm height
com_accel = [0.5, 0.0, 0.0]  # Accelerating forward
zmp = compute_zmp(com_pos, com_accel, foot_positions=[[0, -0.1], [0, 0.1]])
print(f"ZMP: {zmp}")
```

### ZMP-Based Walking Controller

```python
class ZMPWalkingController:
    def __init__(self, step_length=0.2, step_height=0.05):
        self.step_length = step_length
        self.step_height = step_height
        self.phase = 0.0  # 0 to 1 over gait cycle

    def compute_foot_trajectory(self, t, duration=1.0):
        """Generate swing foot trajectory."""
        # Normalized time (0 to 1)
        s = t / duration

        # Swing foot position (parabolic arc)
        x = self.step_length * s
        z = 4 * self.step_height * s * (1 - s)  # Peaks at s=0.5

        return np.array([x, 0, z])

    def compute_torques(self, com_position, com_velocity, zmp_desired):
        """PID control to move CoM towards desired ZMP."""
        zmp_current = compute_zmp(com_position, com_velocity, foot_positions)

        # PD control
        Kp, Kd = 1000, 100
        com_acceleration = (Kp * (zmp_desired - zmp_current) -
                           Kd * com_velocity[:2])

        # Convert to joint torques (simplified)
        # In reality: use inverse dynamics
        torques = self.inverse_dynamics(com_acceleration)
        return torques
```

## Balance Recovery

**Strategies**:
1. **Ankle Strategy**: Rotate about ankle (small disturbances)
2. **Hip Strategy**: Bend at hip (moderate disturbances)
3. **Stepping Strategy**: Take a step (large disturbances)

```python
def balance_recovery(imu_data, threshold_tilt=10.0):
    """Detect imbalance and trigger recovery."""
    tilt_angle = np.degrees(np.arctan2(imu_data.linear_acceleration.y,
                                        imu_data.linear_acceleration.z))

    if abs(tilt_angle) < threshold_tilt:
        return "stable"
    elif abs(tilt_angle) < 20.0:
        return "ankle_strategy"
    elif abs(tilt_angle) < 30.0:
        return "hip_strategy"
    else:
        return "step_strategy"
```

## MoveIt for Manipulation

**MoveIt 2** is a motion planning framework for ROS 2.

### Setup MoveIt

1. **Install**:
```bash
sudo apt install ros-humble-moveit
```

2. **Configure Robot** (URDF + SRDF):
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

3. **Define Planning Groups**: "arm", "gripper", "whole_body"

### Motion Planning Code

```python
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import Pose
import rclpy
from rclpy.action import ActionClient

class ManipulationController:
    def __init__(self, node):
        self.move_group_client = ActionClient(
            node, MoveGroup, '/move_action')

    def plan_to_pose(self, target_pose):
        """Plan arm trajectory to target pose."""
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0

        # Set target pose
        constraint = Constraints()
        constraint.position_constraints.append(
            PositionConstraint(target_pose.position))
        constraint.orientation_constraints.append(
            OrientationConstraint(target_pose.orientation))
        goal.request.goal_constraints.append(constraint)

        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        return future

    def execute_grasp(self, object_pose):
        """Execute pick-and-place sequence."""
        # 1. Pre-grasp: Approach object
        pre_grasp_pose = self.compute_pre_grasp(object_pose)
        self.plan_to_pose(pre_grasp_pose).result()

        # 2. Grasp: Close gripper
        self.close_gripper()

        # 3. Lift: Move upward
        lift_pose = object_pose
        lift_pose.position.z += 0.1
        self.plan_to_pose(lift_pose).result()

        # 4. Place: Move to target
        place_pose = Pose()
        place_pose.position = [0.5, 0.3, 0.1]
        self.plan_to_pose(place_pose).result()

        # 5. Release: Open gripper
        self.open_gripper()
```

## Hands-On Exercise

### Exercise: Implement Grasping Controller with MoveIt

**Objective**: Plan and execute pick-and-place with humanoid arm.

**Steps**:

1. **Launch MoveIt + Simulation**:
```bash
# Terminal 1: Gazebo with humanoid
ros2 launch my_robot_pkg spawn_humanoid.launch.py

# Terminal 2: MoveIt
ros2 launch my_robot_moveit_config demo.launch.py
```

2. **Plan Grasp in RViz**:
   - Open MoveIt RViz plugin
   - Drag interactive marker to object
   - Click "Plan" → "Execute"

3. **Programmatic Grasp**:
```python
# Create node
node = rclpy.create_node('grasp_controller')
controller = ManipulationController(node)

# Target object pose
object_pose = Pose()
object_pose.position = Point(x=0.4, y=0.0, z=0.5)
object_pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

# Execute grasp
controller.execute_grasp(object_pose)
```

4. **Test**:
   - Spawn object (cube) in Gazebo
   - Run grasp script
   - Verify arm reaches, closes gripper, lifts object

**Expected Outcome**: Humanoid arm picks up cube and places it at target location.

**Complete Code**: [GitHub: MoveIt Tutorials](https://github.com/moveit/moveit2_tutorials)

## Summary

- **Bipedal locomotion** requires continuous balance control (ZMP, CoM management)
- **ZMP** must stay within support polygon for stability
- **Balance recovery** uses ankle, hip, or stepping strategies
- **MoveIt** provides motion planning for manipulation tasks
- **Grasping** involves pre-grasp approach, grasp execution, lift, and place

## Next Steps

Continue to [Week 13: Conversational Robotics & Capstone](./week-13-conversational-robotics.md) to integrate voice commands and complete the Autonomous Humanoid project.

---

**Estimated Time**: 6 hours
**Difficulty**: Advanced
