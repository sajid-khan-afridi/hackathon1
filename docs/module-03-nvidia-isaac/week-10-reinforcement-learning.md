---
title: "Week 10: Reinforcement Learning & Sim-to-Real Transfer"
description: "Train RL agents for bipedal locomotion, integrate Nav2 for path planning, and apply sim-to-real transfer techniques."
sidebar_position: 4
week_id: "week-10-reinforcement-learning"
week_number: 10
module: "module-03-nvidia-isaac"
learning_objectives:
  - "Train RL agents for bipedal locomotion in Isaac Sim"
  - "Integrate Nav2 for autonomous navigation"
  - "Apply domain randomization for sim-to-real transfer"
  - "Evaluate trained policies in simulation"
prerequisites:
  - "Weeks 8-9: Isaac SDK, Perception"
  - "Basic RL concepts (optional)"
estimated_time: "6 hours"
difficulty: "advanced"
topics_covered:
  - "Reinforcement learning for locomotion"
  - "Isaac Gym (GPU-accelerated RL)"
  - "Nav2 path planning"
  - "Domain randomization"
  - "Sim-to-real transfer"
hands_on_exercises:
  - title: "Train RL Agent for Bipedal Locomotion"
    description: "Use Isaac Gym to train humanoid walking policy"
    estimated_time: "90 minutes"
    tools_required: ["Isaac Sim", "Isaac Gym", "PyTorch"]
keywords:
  - "reinforcement learning"
  - "bipedal locomotion"
  - "Nav2"
  - "sim-to-real"
  - "domain randomization"
references:
  - title: "Isaac Gym Documentation"
    url: "https://developer.nvidia.com/isaac-gym"
  - title: "Nav2 Documentation"
    url: "https://navigation.ros.org/"
---

# Week 10: Reinforcement Learning & Sim-to-Real Transfer

Train autonomous behaviors using reinforcement learning (RL) in Isaac Gym, integrate Nav2 for navigation, and apply sim-to-real transfer techniques.

## Reinforcement Learning for Locomotion

**RL Framework**: Agent learns through trial-and-error by maximizing rewards.

**Components**:
- **State**: Joint angles, velocities, IMU data
- **Action**: Joint torques or position commands
- **Reward**: Distance traveled, upright posture, energy efficiency

### Isaac Gym for RL

**Isaac Gym** parallelizes thousands of environments on GPU:

```python
from isaacgym import gymapi

# Create Gym instance
gym = gymapi.acquire_gym()

# Create 1024 parallel environments
num_envs = 1024
envs = []
for i in range(num_envs):
    env = gym.create_env(sim, lower, upper, num_per_row)
    envs.append(env)

# Step all environments in parallel (GPU)
gym.simulate(sim)
gym.fetch_results(sim, True)
```

### Training Loop

```python
import torch
from stable_baselines3 import PPO

# Define reward function
def compute_reward(states, actions):
    # Reward forward movement
    reward = states[:, 0]  # X-position

    # Penalize falling
    reward -= torch.where(states[:, 2] < 0.5, 10.0, 0.0)  # Z-position < 0.5m

    # Penalize energy
    reward -= 0.01 * torch.sum(actions**2, dim=1)

    return reward

# Train with PPO
model = PPO("MlpPolicy", env, verbose=1, device="cuda")
model.learn(total_timesteps=10_000_000)
model.save("humanoid_walk_policy")
```

**Training Time**: 2-4 hours on RTX 3080 for basic walking.

## Nav2 Path Planning

**Nav2** provides autonomous navigation for mobile robots:

**Components**:
- **Global Planner**: A*, Dijkstra for long-distance paths
- **Local Planner**: DWA, TEB for obstacle avoidance
- **Costmaps**: 2D grids marking obstacles, inflation zones
- **Behavior Tree**: Task sequencing (navigate, wait, retry)

**Launch Nav2**:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  params_file:=/path/to/nav2_params.yaml
```

**Send Navigation Goal**:
```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# Create action client
nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

# Send goal
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.pose.position.x = 5.0
goal.pose.pose.position.y = 3.0
nav_client.send_goal_async(goal)
```

## Domain Randomization

**Challenge**: Simulated environments differ from reality (lighting, friction, sensor noise).

**Solution**: Randomize simulation parameters during training.

### Randomization Strategy

```python
import random

def randomize_environment(sim):
    # Randomize lighting
    light_intensity = random.uniform(0.5, 1.5)
    gym.set_light_parameters(sim, intensity=light_intensity)

    # Randomize ground friction
    friction = random.uniform(0.5, 1.2)
    gym.set_friction(sim, ground_plane, friction)

    # Add sensor noise
    camera_noise = random.gauss(0, 0.05)  # 5% noise
```

**During Training**: Randomize every episode → Policy learns robust behaviors.

## Sim-to-Real Transfer Checklist

1. **Accurate Dynamics**: Match simulated mass, inertia, friction to real robot
2. **Realistic Sensors**: Add noise, latency, occlusions
3. **Domain Randomization**: Vary lighting, textures, physics
4. **Safety Margins**: Train with conservative limits (max torque, joint limits)
5. **Gradual Deployment**: Test in controlled real-world settings first

## Hands-On Exercise

### Exercise: Train RL Agent for Bipedal Locomotion

**Objective**: Train a humanoid to walk forward using Isaac Gym.

**Steps**:

1. **Install Isaac Gym**:
```bash
# Download Isaac Gym from NVIDIA
# Follow installation instructions
pip install isaacgym
```

2. **Run Pre-built Humanoid Task**:
```bash
cd isaacgym/python/examples
python train.py task=Humanoid num_envs=1024
```

3. **Observe Training**:
   - Watch parallel environments in Isaac Gym viewer
   - Monitor reward curve (should increase over time)
   - Training converges in ~10M timesteps (2-3 hours)

4. **Test Trained Policy**:
```bash
python train.py task=Humanoid test=True checkpoint=humanoid_policy.pth
```

5. **Experiment**:
   - Modify reward function to encourage faster walking
   - Add obstacle avoidance rewards
   - Export policy to ONNX for deployment

**Expected Outcome**: Humanoid learns stable forward walking gait.

**Complete Code**: [GitHub: Isaac Gym Examples](https://github.com/NVIDIA-Omniverse/IsaacGymEnvs)

## Summary

- **Reinforcement learning** trains autonomous behaviors through trial-and-error
- **Isaac Gym** parallelizes RL training on GPU (1000+ envs)
- **Nav2** provides path planning and obstacle avoidance for navigation
- **Domain randomization** improves sim-to-real transfer robustness
- **Sim-to-real transfer** requires accurate dynamics, sensors, and gradual deployment

## Next Steps

Continue to [Module 4: Humanoid Development & Conversational Robotics](../module-04-vla-conversational/) to integrate kinematics, manipulation, and voice control.

---

**Estimated Time**: 6 hours
**Difficulty**: Advanced
