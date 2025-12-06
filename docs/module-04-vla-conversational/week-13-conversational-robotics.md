---
title: "Week 13: Conversational Robotics & Capstone Integration"
description: "Build voice-controlled humanoid robots using OpenAI Whisper for speech recognition and GPT for natural language understanding with ROS 2 action integration."
sidebar_position: 4
week_id: "week-13-conversational-robotics"
week_number: 13
module: "module-04-vla-conversational"
learning_objectives:
  - "Integrate OpenAI Whisper for real-time speech recognition"
  - "Connect GPT models to translate natural language to ROS 2 actions"
  - "Build multi-modal interaction systems (voice + vision + action)"
  - "Design conversational AI pipelines for humanoid robots"
prerequisites:
  - "Weeks 11-12: Kinematics, Locomotion, Manipulation"
  - "All previous modules"
estimated_time: "6 hours"
difficulty: "advanced"
topics_covered:
  - "OpenAI Whisper integration"
  - "GPT for command parsing"
  - "Natural language to ROS 2 actions"
  - "Multi-modal interaction"
  - "Voice-controlled robotics"
hands_on_exercises:
  - title: "Build Voice Command Interface with Whisper"
    description: "Create speech-to-action system for humanoid control"
    estimated_time: "90 minutes"
    tools_required: ["Python", "OpenAI API", "ROS 2 Humble", "Microphone"]
keywords:
  - "conversational AI"
  - "OpenAI Whisper"
  - "GPT"
  - "voice commands"
  - "natural language processing"
  - "multi-modal"
references:
  - title: "OpenAI Whisper"
    url: "https://github.com/openai/whisper"
  - title: "OpenAI API Documentation"
    url: "https://platform.openai.com/docs"
---

# Week 13: Conversational Robotics & Capstone Integration

Build voice-controlled humanoid robots that understand natural language commands and execute complex multi-step tasks. This final week integrates all previous modules into the Autonomous Humanoid Capstone Project.

## Conversational AI Architecture

```
User Voice Command
  ↓
[1] Speech Recognition (Whisper)
  ↓
Text: "Pick up the red cube and place it on the table"
  ↓
[2] Intent Parsing (GPT)
  ↓
Structured Command: {action: "pick_place", object: "red_cube", target: "table"}
  ↓
[3] Task Planning (ROS 2)
  ↓
Action Sequence: [detect_object, navigate, grasp, navigate, place]
  ↓
[4] Execution (MoveIt, Nav2, Isaac ROS)
  ↓
Feedback: "Task completed successfully"
  ↓
[5] Speech Synthesis (TTS)
  ↓
Robot speaks: "I have placed the red cube on the table"
```

## OpenAI Whisper Integration

**Whisper** is an automatic speech recognition (ASR) model from OpenAI.

### Installation

```bash
pip install openai-whisper
# Or use faster whisper.cpp
pip install faster-whisper
```

### Real-Time Speech Recognition

```python
import whisper
import pyaudio
import numpy as np

class VoiceCommandRecognizer:
    def __init__(self, model_size="base"):
        # Load Whisper model
        self.model = whisper.load_model(model_size)  # tiny, base, small, medium, large

        # Audio settings
        self.RATE = 16000
        self.CHUNK = 1024
        self.audio = pyaudio.PyAudio()

    def listen(self, duration=5):
        """Record audio for specified duration."""
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )

        print("Listening...")
        frames = []
        for _ in range(0, int(self.RATE / self.CHUNK * duration)):
            data = stream.read(self.CHUNK)
            frames.append(np.frombuffer(data, dtype=np.int16))

        stream.stop_stream()
        stream.close()

        # Concatenate frames
        audio_data = np.concatenate(frames).astype(np.float32) / 32768.0
        return audio_data

    def transcribe(self, audio_data):
        """Convert speech to text."""
        result = self.model.transcribe(audio_data, language="en")
        return result["text"]

# Usage
recognizer = VoiceCommandRecognizer()
audio = recognizer.listen(duration=5)
text = recognizer.transcribe(audio)
print(f"You said: {text}")
```

## GPT for Command Parsing

Use GPT to extract structured commands from natural language.

```python
import openai

class CommandParser:
    def __init__(self, api_key):
        openai.api_key = api_key

    def parse_command(self, text):
        """Parse natural language into structured robot command."""
        prompt = f"""
        You are a robot command parser. Convert natural language to JSON.

        Examples:
        Input: "Pick up the red cube"
        Output: {{"action": "pick", "object": "red_cube", "location": null}}

        Input: "Navigate to the kitchen"
        Output: {{"action": "navigate", "object": null, "location": "kitchen"}}

        Input: "{text}"
        Output:
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0
        )

        # Parse JSON response
        import json
        command = json.loads(response.choices[0].message.content)
        return command

# Usage
parser = CommandParser(api_key="your-api-key")
command = parser.parse_command("Pick up the blue box and bring it here")
print(command)
# Output: {"action": "pick_place", "object": "blue_box", "location": "user"}
```

## Natural Language to ROS 2 Actions

Map parsed commands to ROS 2 actions.

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from example_interfaces.action import Fibonacci  # Replace with your actions

class RobotCommandExecutor:
    def __init__(self, node):
        self.node = node
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        # Add more action clients (grasp, manipulate, etc.)

    def execute_command(self, command):
        """Execute parsed command using ROS 2 actions."""
        action = command["action"]

        if action == "navigate":
            return self.navigate_to_location(command["location"])

        elif action == "pick":
            return self.pick_object(command["object"])

        elif action == "place":
            return self.place_object(command["location"])

        elif action == "pick_place":
            # Multi-step sequence
            self.pick_object(command["object"])
            self.navigate_to_location(command["location"])
            self.place_object(command["location"])

    def navigate_to_location(self, location):
        """Navigate to named location."""
        # Look up location coordinates (predefined map)
        locations = {
            "kitchen": [5.0, 3.0],
            "table": [2.0, 1.5],
            "user": [0.0, 0.0]
        }

        goal = NavigateToPose.Goal()
        goal.pose.pose.position.x, goal.pose.pose.position.y = locations[location]
        self.nav_client.send_goal_async(goal)

    def pick_object(self, object_name):
        """Grasp specified object."""
        # 1. Detect object with Isaac ROS
        object_pose = self.detect_object(object_name)

        # 2. Plan grasp with MoveIt
        self.plan_and_execute_grasp(object_pose)

    def place_object(self, location):
        """Place held object at location."""
        # Move to location, open gripper
        pass
```

## Multi-Modal Interaction

Combine voice, vision, and action for natural interaction.

```python
class MultiModalRobotController:
    def __init__(self, node):
        self.voice_recognizer = VoiceCommandRecognizer()
        self.command_parser = CommandParser(api_key="your-key")
        self.executor = RobotCommandExecutor(node)

    def run_interaction_loop(self):
        """Main interaction loop."""
        print("Robot ready. Say 'robot' to activate.")

        while True:
            # 1. Wake word detection
            audio = self.voice_recognizer.listen(duration=2)
            text = self.voice_recognizer.transcribe(audio)

            if "robot" in text.lower():
                print("Activated. Listening for command...")

                # 2. Capture full command
                audio = self.voice_recognizer.listen(duration=5)
                text = self.voice_recognizer.transcribe(audio)
                print(f"Command: {text}")

                # 3. Parse command
                command = self.command_parser.parse_command(text)
                print(f"Parsed: {command}")

                # 4. Execute command
                self.executor.execute_command(command)

                # 5. Provide feedback
                print("Task completed!")
```

## Hands-On Exercise

### Exercise: Build Voice Command Interface

**Objective**: Create speech-to-action system for humanoid control.

**Steps**:

1. **Install Dependencies**:
```bash
pip install openai-whisper pyaudio openai
```

2. **Create Voice Controller Node**:
```python
import rclpy
from rclpy.node import Node

class VoiceControllerNode(Node):
    def __init__(self):
        super().__init__('voice_controller')
        self.controller = MultiModalRobotController(self)

    def run(self):
        self.controller.run_interaction_loop()

def main():
    rclpy.init()
    node = VoiceControllerNode()
    node.run()

if __name__ == '__main__':
    main()
```

3. **Test Commands**:
   - "Robot, pick up the red cube"
   - "Robot, navigate to the kitchen"
   - "Robot, wave your hand"

4. **Extend**:
   - Add error handling for unrecognized commands
   - Implement confirmation ("Did you mean X?")
   - Add visual feedback (LED, screen display)

**Expected Outcome**: Robot responds to voice commands and executes actions.

**Complete Code**: [GitHub: Voice-Controlled Robot Examples](https://github.com/ros2/examples)

## Capstone Integration

This week connects to the **Autonomous Humanoid Capstone Project**:

**System Overview**:
1. **Voice Input**: User says "Pick up the cup and bring it to me"
2. **Command Parsing**: GPT extracts structured command
3. **Perception**: Isaac ROS detects cup with DOPE
4. **Navigation**: Nav2 plans path to cup
5. **Manipulation**: MoveIt grasps cup
6. **Return**: Nav2 navigates back to user
7. **Feedback**: "I have brought you the cup"

See [Capstone Project](../capstone/) for full integration guide.

## Summary

- **OpenAI Whisper** enables real-time speech recognition for robot control
- **GPT** parses natural language into structured robot commands
- **ROS 2 actions** execute multi-step tasks (navigation, manipulation)
- **Multi-modal interaction** combines voice, vision, and physical action
- **Capstone project** integrates all 13 weeks of learning into autonomous humanoid system

## Next Steps

Proceed to the [Autonomous Humanoid Capstone Project](../capstone/) to build your complete system integrating ROS 2, simulation, AI perception, and conversational control.

**Congratulations on completing all 13 weeks!** 🎉

---

**Estimated Time**: 6 hours
**Difficulty**: Advanced
