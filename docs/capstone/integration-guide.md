---
title: "Capstone Integration Guide"
description: "Step-by-step guide to integrate all 4 modules into the Autonomous Humanoid Capstone Project."
sidebar_position: 3
---

# Capstone Integration Guide

This guide provides step-by-step instructions to integrate ROS 2, simulation, AI perception, and conversational control into the complete Autonomous Humanoid system.

## Prerequisites Checklist

Before starting integration, verify:
- ✅ Completed Modules 1-4 (Weeks 1-13)
- ✅ ROS 2 Humble installed and tested
- ✅ Isaac Sim OR Gazebo installed
- ✅ MoveIt 2 configured for humanoid
- ✅ Nav2 installed and tested
- ✅ OpenAI API key obtained
- ✅ GPU available (local or cloud)

## Phase 1: Workspace Setup

### Step 1.1: Create ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/capstone_ws/src
cd ~/capstone_ws/src

# Clone starter code (placeholder - replace with actual repo)
git clone https://github.com/example/capstone-starter.git

# Install dependencies
cd ~/capstone_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Step 1.2: Configure Environment

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/capstone_ws/install/setup.bash" >> ~/.bashrc
echo "export OPENAI_API_KEY='your-key-here'" >> ~/.bashrc
source ~/.bashrc
```

### Step 1.3: Build Workspace

```bash
cd ~/capstone_ws
colcon build --symlink-install
source install/setup.bash
```

**Checkpoint**: `ros2 pkg list | grep capstone` shows your packages.

## Phase 2: Simulation Environment

### Step 2.1: Launch Isaac Sim with Humanoid

```bash
# Option A: Isaac Sim
ros2 launch capstone_bringup isaac_sim.launch.py

# Option B: Gazebo
ros2 launch capstone_bringup gazebo.launch.py
```

**Expected**: Humanoid spawns in environment with table, objects, obstacles.

### Step 2.2: Verify Sensor Data

```bash
# Check camera feed
ros2 topic hz /camera/rgb/image_raw
ros2 topic hz /camera/depth/image_raw

# Check LiDAR
ros2 topic hz /scan

# Check IMU
ros2 topic hz /imu/data

# Check joint states
ros2 topic hz /joint_states
```

**Checkpoint**: All sensors publishing at expected rates (10-30 Hz).

### Step 2.3: Visualize in RViz

```bash
ros2 launch capstone_bringup rviz.launch.py
```

**Configure RViz**:
- Add RobotModel → Shows humanoid
- Add Image → Display camera feed
- Add LaserScan → Display LiDAR
- Add TF → Show coordinate frames

## Phase 3: Perception Pipeline

### Step 3.1: Launch Isaac ROS DOPE

```bash
# Start object detection
ros2 launch capstone_perception object_detection.launch.py
```

**Configuration** (`config/dope_params.yaml`):
```yaml
object_detection:
  ros__parameters:
    model_path: "/path/to/dope_model.pth"
    object_classes: ["cube", "cup", "bottle"]
    confidence_threshold: 0.7
    detection_rate: 10.0  # Hz
```

### Step 3.2: Test Object Detection

```bash
# Place objects in simulation
# Run detection node
ros2 launch capstone_perception object_detection.launch.py

# Check detections
ros2 topic echo /detected_objects
```

**Expected Output**:
```
detected_objects:
  - class: "red_cube"
    pose: {position: {x: 2.0, y: 1.0, z: 0.5}, ...}
    confidence: 0.92
```

**Checkpoint**: Objects detected with >80% confidence.

### Step 3.3: Launch VSLAM

```bash
ros2 launch capstone_perception vslam.launch.py
```

**Verify Localization**:
```bash
ros2 topic echo /visual_slam/tracking/odometry
```

## Phase 4: Navigation

### Step 4.1: Configure Nav2

**Edit** `config/nav2_params.yaml`:
```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5
      max_vel_theta: 1.0
```

### Step 4.2: Create Navigation Map

```bash
# Build map from SLAM
ros2 launch capstone_navigation slam_toolbox.launch.py

# Drive robot around environment
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/capstone_ws/maps/office_map
```

### Step 4.3: Test Navigation

```bash
# Launch Nav2
ros2 launch capstone_navigation navigation.launch.py map:=~/capstone_ws/maps/office_map.yaml

# Send navigation goal (RViz)
# Click "2D Goal Pose" → Click target location

# Or programmatically
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}"
```

**Checkpoint**: Robot navigates to goal, avoids obstacles.

## Phase 5: Manipulation

### Step 5.1: Configure MoveIt

```bash
# Launch MoveIt
ros2 launch capstone_manipulation moveit.launch.py
```

**Test IK** (RViz MoveIt plugin):
- Drag interactive marker to target
- Verify reachability
- Click "Plan & Execute"

### Step 5.2: Grasp Planning

**Create Grasp Planner Node** (`grasp_planner.py`):
```python
from geometry_msgs.msg import Pose

def compute_grasp_pose(object_pose):
    """Compute top-down grasp."""
    grasp = Pose()
    grasp.position = object_pose.position
    grasp.position.z += 0.1  # Approach from above
    grasp.orientation.x = 1.0  # Vertical
    return grasp
```

### Step 5.3: Test Pick-and-Place

```bash
# Run manipulation test
ros2 run capstone_manipulation test_pick_place.py --object red_cube --target table
```

**Expected**: Arm reaches object, closes gripper, lifts, moves to target, places.

**Checkpoint**: 80%+ grasp success rate.

## Phase 6: Voice Control

### Step 6.1: Setup Whisper

```python
# voice_controller.py
import whisper

model = whisper.load_model("base")

def listen_for_command(duration=5):
    # Record audio
    audio = record_audio(duration)
    # Transcribe
    result = model.transcribe(audio)
    return result["text"]
```

### Step 6.2: Setup GPT Parser

```python
import openai

def parse_command(text):
    prompt = f"""Parse robot command:
    Input: "{text}"
    Output JSON: {{"action": "...", "object": "...", "location": "..."}}
    """

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )

    return json.loads(response.choices[0].message.content)
```

### Step 6.3: Test Voice Pipeline

```bash
# Start voice controller
ros2 run capstone_voice voice_controller.py

# Speak command: "Pick up the red cube"
# Verify: Command parsed and executed
```

**Checkpoint**: Voice commands trigger correct actions.

## Phase 7: Full Integration

### Step 7.1: High-Level Task Planner

**Create State Machine** (`task_planner.py`):
```python
class TaskPlanner:
    def execute_pick_place(self, object_name, target_location):
        # State 1: Detect object
        object_pose = self.detect_object(object_name)

        # State 2: Navigate to object
        self.navigate_to(object_pose.position)

        # State 3: Grasp object
        self.grasp_object(object_pose)

        # State 4: Navigate to target
        self.navigate_to(target_location)

        # State 5: Place object
        self.place_object()

        # State 6: Return feedback
        return "Task completed successfully"
```

### Step 7.2: Launch Full System

**Master Launch File** (`capstone.launch.py`):
```python
def generate_launch_description():
    return LaunchDescription([
        # Simulation
        IncludeLaunchDescription('isaac_sim.launch.py'),

        # Perception
        IncludeLaunchDescription('object_detection.launch.py'),
        IncludeLaunchDescription('vslam.launch.py'),

        # Navigation
        IncludeLaunchDescription('navigation.launch.py'),

        # Manipulation
        IncludeLaunchDescription('moveit.launch.py'),

        # Voice Control
        Node(package='capstone_voice', executable='voice_controller'),

        # Task Planner
        Node(package='capstone_planning', executable='task_planner'),
    ])
```

**Launch**:
```bash
ros2 launch capstone_bringup capstone.launch.py
```

### Step 7.3: Run Integration Test

```bash
# Test Scenario 1: Simple pick
# User: "Pick up the red cube"
# Expected: Robot detects, navigates, grasps, lifts

# Test Scenario 2: Pick and place
# User: "Pick up the blue cup and place it on the table"
# Expected: Full pick-place sequence

# Test Scenario 3: Bring to user
# User: "Bring me the green bottle"
# Expected: Pick bottle, navigate to user, hand over
```

## Phase 8: Testing & Validation

### Step 8.1: Automated Tests

```bash
# Run integration tests
ros2 test capstone_integration_tests

# Run perception tests
ros2 test capstone_perception_tests

# Run manipulation tests
ros2 test capstone_manipulation_tests
```

### Step 8.2: Manual Test Scenarios

See [Requirements](./requirements.md) for 5 test scenarios. Execute each and record:
- Success/failure
- Error messages
- Completion time
- Issues encountered

### Step 8.3: Record Demo Video

```bash
# Record screen
ros2 bag record -a -o ~/capstone_demo

# Or use OBS Studio for screen capture
```

**Video Structure**:
1. Introduction (30s): Overview of system
2. Demonstration (2min): Show 3-5 test scenarios
3. Conclusion (30s): Highlight achievements, limitations

## Troubleshooting

### Issue: Objects Not Detected

**Solution**:
- Check camera feed (`ros2 topic echo /camera/rgb/image_raw`)
- Verify DOPE model loaded (`ros2 param get /object_detection model_path`)
- Check confidence threshold (lower if needed)
- Ensure objects in camera field of view

### Issue: Navigation Fails

**Solution**:
- Verify map loaded (`ros2 topic echo /map`)
- Check costmap (`ros2 topic echo /global_costmap/costmap`)
- Increase planner timeout
- Check for TF errors (`ros2 run tf2_tools view_frames`)

### Issue: Grasp Fails

**Solution**:
- Verify IK solution exists (MoveIt RViz plugin)
- Check collision scene updated
- Adjust grasp approach height
- Increase gripper force

### Issue: Voice Not Recognized

**Solution**:
- Test microphone (`arecord -d 5 test.wav && aplay test.wav`)
- Check Whisper model loaded
- Reduce background noise
- Increase recording duration

## Next Steps

After completing integration:
1. **Optimize**: Tune parameters for better performance
2. **Extend**: Add new commands, objects, locations
3. **Document**: Write technical report
4. **Share**: Present to community, publish on GitHub

---

**Congratulations! You've built a complete autonomous humanoid system.** 🎉

For support, see [Capstone FAQ](https://example.com/capstone-faq) or join [Discord Community](https://discord.gg/example).
