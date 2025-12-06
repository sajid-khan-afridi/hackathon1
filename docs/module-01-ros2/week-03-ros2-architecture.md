---
title: "Week 3: ROS 2 Architecture"
description: "Explore ROS 2 architecture, DDS middleware, and the publish-subscribe communication pattern for distributed robotic systems."
sidebar_position: 4
week_id: "week-03-ros2-architecture"
week_number: 3
module: "module-01-ros2"
learning_objectives:
  - "Explain ROS 2 architecture and its advantages over ROS 1"
  - "Understand DDS (Data Distribution Service) middleware and QoS policies"
  - "Identify key ROS 2 concepts: nodes, topics, services, actions"
  - "Distinguish between communication patterns and their use cases"
prerequisites:
  - "Week 1: Introduction to Physical AI"
  - "Week 2: Embodied Intelligence"
estimated_time: "3 hours"
difficulty: "intermediate"
topics_covered:
  - "ROS 2 vs ROS 1 comparison"
  - "DDS middleware layer"
  - "Nodes and node composition"
  - "Topics (publish-subscribe)"
  - "Services (request-reply)"
  - "Actions (goal-based)"
  - "Quality of Service (QoS) policies"
hands_on_exercises:
  - title: "Inspect ROS 2 Nodes Using CLI Tools"
    description: "Use ros2 CLI commands to explore running nodes and their communication"
    estimated_time: "30 minutes"
    tools_required: ["ROS 2 Humble", "Ubuntu 22.04 or Docker"]
keywords:
  - "ROS 2"
  - "DDS"
  - "middleware"
  - "nodes"
  - "topics"
  - "services"
  - "actions"
  - "QoS"
  - "publish-subscribe"
references:
  - title: "ROS 2 Design Documentation"
    url: "https://design.ros2.org/"
  - title: "DDS vs ROS 1 Communication"
    url: "https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html"
---

# Week 3: ROS 2 Architecture

Welcome to **Robot Operating System 2 (ROS 2)**—the middleware framework that powers modern autonomous robots. This week introduces ROS 2's architecture, communication patterns, and the DDS middleware that enables distributed robotic systems.

## What is ROS 2?

**ROS 2** is an open-source middleware framework for building robot software. It provides:
- **Communication infrastructure**: Nodes exchange data via topics, services, and actions
- **Hardware abstraction**: Unified interfaces for sensors, actuators, and controllers
- **Package ecosystem**: Thousands of pre-built libraries (navigation, perception, simulation)
- **Tools**: CLI utilities, visualization (RViz), simulation (Gazebo integration)

### ROS 1 vs. ROS 2: Why the Upgrade?

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Communication** | Custom TCP/UDP (TCPROS) | DDS (Data Distribution Service) |
| **Real-time** | No guarantees | Real-time capable with DDS |
| **Security** | None | DDS-Security (encryption, authentication) |
| **Multi-robot** | Difficult (master bottleneck) | Native support (decentralized) |
| **Platforms** | Linux only | Linux, Windows, macOS, RTOS |
| **Python** | Python 2 (EOL) | Python 3 |
| **Lifecycle** | No node lifecycle | Managed node lifecycle |

**Key Improvement**: ROS 2 eliminates the single-point-of-failure "master" node from ROS 1, enabling truly distributed systems.

## DDS: The Middleware Foundation

**DDS (Data Distribution Service)** is the communication layer underlying ROS 2. It provides:

1. **Peer-to-Peer Communication**: No central broker (unlike ROS 1's master)
2. **Quality of Service (QoS)**: Configure reliability, latency, durability per-topic
3. **Discovery**: Nodes automatically find each other on the network
4. **Type Safety**: Strongly-typed messages with schema validation

### DDS Vendors

ROS 2 supports multiple DDS implementations (middleware vendors):
- **Fast DDS** (default): eProsima, good performance
- **CycloneDDS**: Eclipse, lightweight, low-latency
- **Connext DDS**: RTI, industrial-grade, certified for safety-critical systems

**Why Multiple Vendors?**: Different applications need different trade-offs (latency vs. throughput, resource usage, certification requirements).

### Quality of Service (QoS) Policies

QoS policies let you tune communication behavior:

| Policy | Options | Use Case |
|--------|---------|----------|
| **Reliability** | RELIABLE, BEST_EFFORT | Sensor data (BEST_EFFORT) vs. commands (RELIABLE) |
| **Durability** | TRANSIENT_LOCAL, VOLATILE | Late-joining subscribers get recent messages |
| **History** | KEEP_LAST(N), KEEP_ALL | Buffer last N messages |
| **Deadline** | Duration | Detect if publisher stops sending data |
| **Lifespan** | Duration | Discard old messages after timeout |

**Example**: A camera publishes images at 30 FPS with BEST_EFFORT (okay to drop frames), but a robot arm subscribes to commands with RELIABLE (must execute every command).

## Core ROS 2 Concepts

### 1. Nodes

A **node** is an executable process that performs a specific task. Examples:
- Camera driver node (publishes images)
- Object detector node (subscribes to images, publishes bounding boxes)
- Motor controller node (subscribes to velocity commands, controls motors)

**Design Principle**: One node per responsibility (modularity).

**Lifecycle Nodes**: Can be started, paused, stopped, and reconfigured dynamically.

### 2. Topics (Publish-Subscribe)

**Topics** are named buses for streaming data. Publishers send messages, subscribers receive them.

**Characteristics**:
- **Many-to-many**: Multiple publishers and subscribers per topic
- **Asynchronous**: Publish doesn't wait for subscribers
- **Decoupled**: Publishers don't know who's subscribed

**Example**:
```
/camera/image_raw (sensor_msgs/Image)
  Publisher: camera_driver_node
  Subscribers: object_detector_node, display_node, logger_node
```

**When to Use**: Continuous data streams (sensor readings, state updates).

### 3. Services (Request-Reply)

**Services** provide synchronous request-response communication.

**Characteristics**:
- **One-to-one**: Client sends request, server sends response
- **Synchronous**: Client blocks until response received
- **Temporary**: No persistent connection

**Example**:
```
/add_two_ints (example_interfaces/AddTwoInts)
  Request: {a: 5, b: 3}
  Response: {sum: 8}
```

**When to Use**: Infrequent operations (configuration changes, one-time queries).

### 4. Actions (Goal-Based)

**Actions** handle long-running tasks with feedback and cancellation.

**Characteristics**:
- **Three communication channels**:
  - **Goal**: Client sends goal to server
  - **Feedback**: Server sends progress updates to client
  - **Result**: Server sends final outcome when done
- **Cancellable**: Client can abort goal mid-execution
- **Asynchronous**: Client doesn't block

**Example**:
```
/navigate_to_pose (nav2_msgs/NavigateToPose)
  Goal: {pose: {x: 10.0, y: 5.0, theta: 0.0}}
  Feedback: {distance_remaining: 2.5m}
  Result: {success: true}
```

**When to Use**: Navigation, manipulation, any multi-step operation.

## ROS 2 Communication Patterns

| Pattern | Example Use Case | Latency | Throughput |
|---------|------------------|---------|------------|
| **Topic** | Camera → Image processing | Low | High |
| **Service** | Get robot status | Medium | Low |
| **Action** | Navigate to goal | Medium | Medium |
| **Parameter** | Configure node settings | Low | Very Low |

## Hands-On Exercise

### Exercise: Inspect ROS 2 Nodes Using CLI Tools

**Objective**: Explore running ROS 2 nodes and their communication using command-line tools.

**Tools Required**:
- ROS 2 Humble installed
- Ubuntu 22.04 (or Docker with ROS 2 Humble image)

**Steps**:

1. **Install ROS 2 Humble** (if not already installed):
```bash
# Ubuntu 22.04
sudo apt update && sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

2. **Run Demo Nodes**:
```bash
# Terminal 1: Start talker node (publishes to /chatter topic)
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener node (subscribes to /chatter)
ros2 run demo_nodes_cpp listener
```

3. **Inspect Node Graph**:
```bash
# List all running nodes
ros2 node list

# Get info about talker node
ros2 node info /talker

# Output shows:
#   Subscribers: none
#   Publishers: /chatter (std_msgs/msg/String)
#   Services: ...
```

4. **Explore Topics**:
```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /chatter

# Echo messages (view live data)
ros2 topic echo /chatter

# Measure publication rate
ros2 topic hz /chatter
```

5. **Visualize Graph**:
```bash
# Install rqt_graph (if not installed)
sudo apt install ros-humble-rqt-graph

# Launch graphical node/topic visualizer
rqt_graph
```

**Expected Outcome**: You'll see `/talker` publishing to `/chatter` and `/listener` subscribing to it. The graph visualizes the data flow.

**Reflection Questions**:
- What happens if you kill the talker? Does the listener crash?
- Can you run multiple listener nodes? What happens?
- How would you publish a message to `/chatter` manually from CLI?

**Extension**: Explore a more complex example:
```bash
ros2 launch turtlesim_node turtlesim_node
ros2 run turtlesim turtle_teleop_key
```
Use `ros2 node info`, `ros2 topic list`, and `rqt_graph` to understand the turtlesim communication architecture.

## Summary

In this week, you learned:

- **ROS 2** is a middleware framework for building distributed robotic systems, improving on ROS 1
- **DDS middleware** provides peer-to-peer communication, QoS policies, and multi-vendor support
- **Nodes** are independent processes that communicate via topics, services, and actions
- **Topics** enable asynchronous publish-subscribe for streaming data
- **Services** provide synchronous request-reply for one-time operations
- **Actions** handle long-running goals with feedback and cancellation
- **CLI tools** (`ros2 node`, `ros2 topic`, `rqt_graph`) help inspect and debug ROS systems

## Next Steps

Continue to [Week 4: ROS 2 Nodes, Topics, Services & Actions](./week-04-ros2-nodes-topics.md) to learn how to create your own ROS 2 nodes and implement communication patterns in Python.

---

**Estimated Time**: 3 hours (reading + exercise)
**Difficulty**: Intermediate
**Prerequisites**: Weeks 1-2
