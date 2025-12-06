---
title: "Week 4: Creating ROS 2 Nodes, Topics, Services & Actions"
description: "Build ROS 2 nodes in Python, implement publishers/subscribers, services, and action servers for humanoid robot communication."
sidebar_position: 5
week_id: "week-04-ros2-nodes-topics"
week_number: 4
module: "module-01-ros2"
learning_objectives:
  - "Create ROS 2 nodes using rclpy in Python"
  - "Implement publishers and subscribers for topic communication"
  - "Develop service servers and clients for request-reply patterns"
  - "Build action servers with feedback and cancellation support"
prerequisites:
  - "Week 3: ROS 2 Architecture"
  - "Python classes and object-oriented programming"
estimated_time: "4 hours"
difficulty: "intermediate"
topics_covered:
  - "rclpy Python library"
  - "Node class and inheritance"
  - "Publishers and subscribers"
  - "Service servers and clients"
  - "Action servers and clients"
  - "Timers and callbacks"
hands_on_exercises:
  - title: "Create a Minimal ROS 2 Publisher Node in Python"
    description: "Write a Python node that publishes messages to a topic at 1 Hz"
    estimated_time: "45 minutes"
    tools_required: ["ROS 2 Humble", "Python 3.10+", "VS Code"]
keywords:
  - "rclpy"
  - "publishers"
  - "subscribers"
  - "services"
  - "actions"
  - "Python"
  - "callbacks"
references:
  - title: "rclpy API Documentation"
    url: "https://docs.ros2.org/latest/api/rclpy/"
  - title: "Writing a Simple Publisher and Subscriber (Python)"
    url: "https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html"
---

# Week 4: Creating ROS 2 Nodes, Topics, Services & Actions

This week you'll transition from theory to practice by building ROS 2 nodes in Python. You'll create publishers, subscribers, services, and action servers—the building blocks of any robotic system.

## The rclpy Library

**rclpy** is the Python client library for ROS 2. It provides:
- `Node` class for creating nodes
- Publisher/Subscriber classes for topics
- Service/Client classes for services
- ActionServer/ActionClient for actions
- Timer, Parameter, Logger utilities

### Minimal Node Example

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')  # Node name
        self.get_logger().info('Node has started')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)  # Keep node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Components**:
- `rclpy.init()`: Initialize ROS 2 context
- `Node` class: Base class for all nodes
- `rclpy.spin()`: Process callbacks (keeps node alive)
- `rclpy.shutdown()`: Clean up resources

## Publishers and Subscribers

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        # Create publisher: topic name, message type, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # Timer: call publish_message() every 1.0 seconds
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main():
    rclpy.init()
    node = TalkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        # Create subscriber: topic, message type, callback function
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()
    node = ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Pattern**: Subscribers register callback functions that execute when messages arrive.

## Services (Request-Reply)

### Service Server

```python
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

### Service Client

```python
class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.cli.call_async(request)  # Async call
        return future
```

## Actions (Goal-Based Communication)

### Action Server

```python
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Hands-On Exercise

### Exercise: Create a Minimal ROS 2 Publisher Node

**Objective**: Build a Python node that publishes sensor data to a topic.

**Steps**:

1. **Create Package**:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
cd my_robot_pkg/my_robot_pkg
```

2. **Write Publisher Node** (`sensor_publisher.py`):
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(Range, 'sensor/range', 10)
        self.timer = self.create_timer(0.5, self.publish_sensor_data)

    def publish_sensor_data(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'
        msg.range = 2.5  # meters
        msg.min_range = 0.1
        msg.max_range = 10.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing range: {msg.range}m')

def main():
    rclpy.init()
    node = SensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

3. **Build and Run**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash
ros2 run my_robot_pkg sensor_publisher
```

4. **Test**:
```bash
# In another terminal
ros2 topic echo /sensor/range
ros2 topic hz /sensor/range
```

**Expected Outcome**: Sensor data published at 2 Hz, visible via `ros2 topic echo`.

**Complete Code**: [GitHub: ros2-sensor-publisher](https://github.com/ros2/examples/tree/humble/rclpy/topics)

## Summary

In this week, you learned:
- **rclpy** is the Python library for ROS 2 development
- **Publishers** send messages to topics, **subscribers** receive them
- **Services** provide request-reply communication for one-time operations
- **Actions** handle long-running goals with feedback and cancellation
- **Timers** enable periodic callbacks for publishing sensor data

## Next Steps

Continue to [Week 5: Python Packages & URDF](./week-05-ros2-python-urdf.md) to structure ROS 2 projects and define humanoid robot models.

---

**Estimated Time**: 4 hours
**Difficulty**: Intermediate
