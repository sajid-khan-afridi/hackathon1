---
title: "Week 9: AI-Powered Perception & Manipulation"
description: "Implement computer vision, object detection, VSLAM, and manipulation controllers using Isaac ROS and deep learning models."
sidebar_position: 3
week_id: "week-09-ai-perception"
week_number: 9
module: "module-03-nvidia-isaac"
learning_objectives:
  - "Implement object detection with Isaac ROS DOPE"
  - "Build visual SLAM systems with Isaac ROS VSLAM"
  - "Create grasping controllers with manipulation primitives"
  - "Deploy deep learning models for perception"
prerequisites:
  - "Week 8: Isaac SDK & Isaac Sim"
  - "Computer vision basics"
estimated_time: "5 hours"
difficulty: "advanced"
topics_covered:
  - "Isaac ROS packages"
  - "Object detection (DOPE)"
  - "Visual SLAM"
  - "Manipulation primitives"
  - "Grasping strategies"
hands_on_exercises:
  - title: "Implement Object Detection with Isaac ROS"
    description: "Run DOPE object detector on simulated camera feeds"
    estimated_time: "60 minutes"
    tools_required: ["Isaac Sim", "Isaac ROS", "ROS 2 Humble"]
keywords:
  - "object detection"
  - "VSLAM"
  - "perception"
  - "manipulation"
  - "Isaac ROS"
references:
  - title: "Isaac ROS Documentation"
    url: "https://nvidia-isaac-ros.github.io/"
---

# Week 9: AI-Powered Perception & Manipulation

This week focuses on AI-driven perception (computer vision, SLAM) and manipulation (grasping, pick-and-place) using Isaac ROS packages optimized for NVIDIA GPUs.

## Isaac ROS Packages

**Isaac ROS** provides GPU-accelerated perception libraries:

| Package | Function | Use Case |
|---------|----------|----------|
| **isaac_ros_dope** | 6-DOF object pose estimation | Pick-and-place |
| **isaac_ros_vslam** | Visual SLAM | Navigation, mapping |
| **isaac_ros_depth_segmentation** | Depth-based segmentation | Obstacle detection |
| **isaac_ros_apriltag** | Fiducial marker detection | Calibration, localization |
| **isaac_ros_image_proc** | Image processing | Rectification, debayering |

## Object Detection with DOPE

**DOPE (Deep Object Pose Estimation)** detects objects and estimates their 6-DOF poses.

**Setup**:
```bash
# Install Isaac ROS DOPE
sudo apt install ros-humble-isaac-ros-dope

# Download pre-trained model
wget https://nvidia-isaac-ros.github.io/models/dope_model.pth
```

**ROS 2 Node**:
```python
from isaac_ros_dope import DopeNode

# Launch DOPE detector
ros2 run isaac_ros_dope isaac_ros_dope --ros-args \
  -p model_file_path:=/path/to/dope_model.pth \
  -p object_name:=soup_can
```

**Input**: RGB image (`sensor_msgs/Image`)
**Output**: Object poses (`geometry_msgs/PoseArray`)

## Visual SLAM

**Isaac ROS VSLAM** builds 3D maps from camera data:

```bash
# Launch VSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**Features**:
- Loop closure detection
- Pose graph optimization
- Real-time performance (30+ FPS)

**Visualization**:
```bash
ros2 run rviz2 rviz2
# Add: Map, Camera, Odometry displays
```

## Manipulation Primitives

**Grasping Workflow**:
1. **Perceive**: Detect object with DOPE
2. **Plan**: Compute grasp pose
3. **Execute**: Move arm to grasp
4. **Verify**: Check grasp success (force sensors)

### Grasp Pose Calculation

```python
import numpy as np
from geometry_msgs.msg import Pose

def compute_top_grasp(object_pose):
    """Compute top-down grasp pose for object."""
    grasp_pose = Pose()
    grasp_pose.position = object_pose.position
    grasp_pose.position.z += 0.1  # Approach from 10cm above

    # Orient gripper downward
    grasp_pose.orientation.x = 1.0  # Quaternion for vertical orientation
    grasp_pose.orientation.w = 0.0

    return grasp_pose
```

### MoveIt Integration

```python
from moveit_msgs.action import MoveGroup

# Plan to grasp pose
move_group_client.send_goal(target_pose=grasp_pose)

# Wait for execution
move_group_client.wait_for_result()

# Close gripper
gripper_command.publish(position=0.0)  # Fully closed
```

## Hands-On Exercise

### Exercise: Implement Object Detection with Isaac ROS

**Objective**: Run DOPE detector on Isaac Sim camera feed.

**Steps**:

1. **Launch Isaac Sim with Camera**:
```python
# Isaac Sim script
from omni.isaac.core import World
from omni.isaac.sensor import Camera

world = World()
camera = Camera(prim_path="/World/Camera", resolution=(640, 480))
camera.initialize()
```

2. **Start ROS 2 Bridge**:
```bash
ros2 launch isaac_ros_isaac_sim_bridge isaac_sim_bridge.launch.py
```

3. **Run DOPE Detector**:
```bash
ros2 run isaac_ros_dope isaac_ros_dope --ros-args \
  -p model_file_path:=/models/dope_soup.pth \
  -r /image:=/camera/rgb/image_raw
```

4. **Visualize Detections**:
```bash
ros2 run rviz2 rviz2
# Add: Image, PoseArray displays
```

**Expected Outcome**: Bounding boxes and 6-DOF poses overlaid on camera feed.

**Complete Code**: [GitHub: Isaac ROS Examples](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection)

## Summary

- **Isaac ROS** provides GPU-accelerated perception libraries
- **DOPE** estimates 6-DOF object poses for manipulation
- **VSLAM** builds 3D maps for navigation
- **Manipulation primitives** combine perception + motion planning for grasping
- **MoveIt integration** enables complex arm trajectories

## Next Steps

Continue to [Week 10: Reinforcement Learning & Sim-to-Real](./week-10-reinforcement-learning.md) to train autonomous behaviors with RL.

---

**Estimated Time**: 5 hours
**Difficulty**: Advanced
