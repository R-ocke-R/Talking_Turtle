# Talking_Turtle - Voice-Controlled Turtle Movement

A ROS 2 package that enables voice control of turtlesim with speech recognition and text-to-speech feedback.

---

## Overview

The `ninja_turtle` package provides an interactive way to control turtle movement in turtlesim using voice commands. The system combines speech recognition with real-time turtle control, offering both voice-activated and manual input modes for comprehensive turtle navigation.

---

## Core Features

- **Voice Recognition:** Real-time speech-to-command conversion using Google Speech API
- **Text-to-Speech Feedback:** Audio confirmation of recognized commands by the node itself
- **Multiple Input Methods:** Voice control and manual command input using `command_publisher.py`
- **Offline Speech Recognition:** Alternative implementation for offline use using pyttsx3, sounddevice, and Vosk

---

## System Architecture

```
Voice Input → Speech Recognition → Command Processing → Turtle Movement
```

The package follows a modular ROS 2 architecture with separate nodes for speech processing and movement control.

---

## Package Components

### 1. `voice_listener.py` - Speech Recognition Node
- **Primary voice control interface**
- **Speech Engine:** Google Speech API (upgraded from Sphinx)
- **Supported Commands:**
  - `up`, `down` - Linear movement
  - `left`, `right` - Angular rotation
  - `stop` - Halt movement
  - `arc`, `circle` - Pattern movements (currently both draw an arc)
  - `reset` - Position reset
  - `clear` - Clear lines on the screen
- **Audio Feedback System:** Turtle greets the user upon startup and gives command confirmation
- **Output Topic:** `/turtle_cmd`

### 2. `turtle_controller.py` - Movement Execution Node
- **Translates commands into turtle movements**
- **Input:** Commands from `/turtle_cmd` topic
- **Movement Types:**
  - Basic: Linear (`up`/`down`) and angular (`left`/`right`) movements
  - Patterns: `arc` and `circle` (currently both draw an arc)
- **Output:** Velocity commands to `/turtle1/cmd_vel`

### 3. `command_publisher.py` - Manual Input Node
- **Interactive command-line interface**
- **Purpose:** Testing and manual control without speech recognition
- **Interface:** Text-based command input
- **Use Cases:** Debugging, environments without microphone access

### 4. `dummy_voice_listener.py` - Alternative Speech Engine
- **Offline speech recognition implementation**
- **Engine:** Vosk for offline processing
- **Features:** Audio debugging capabilities
- **Benefits:** Works without internet connectivity

---

## Installation & Dependencies

### Environment Setup
```bash
# Uses Pixi environment management
pixi install
```

### Python Dependencies
- `speech_recognition` - Speech-to-text processing (for `voice_listener.py`)
- `pyttsx3` - Text-to-speech synthesis
- Standard ROS 2 Python libraries

### ROS Dependencies
- `geometry_msgs` - Velocity message types
- `turtlesim` - Turtle simulation environment

---

## Usage Instructions

### Launch the System
Source your ROS environment. For Mac/robostack users:

```bash
pixi shell -e humble
```

Build the workspace:
```bash
pixi run -e humble build
```

Then launch all nodes:
```bash
ros2 launch ninja_turtle launch_ninja_turtle.py
```

### Alternative Usage Modes
- **Manual command input:**
  ```bash
  ros2 run ninja_turtle command_publisher
  ```
- **Offline voice recognition:**
  ```bash
  ros2 run ninja_turtle dummy_voice_listener
  ```

---

## Voice Commands Reference

| Command   | Action           | Duration    |
|-----------|------------------|-------------|
| up        | Move forward     | Continuous  |
| down      | Move backward    | Continuous  |
| left      | Rotate left      | Continuous  |
| right     | Rotate right     | Continuous  |
| stop      | Stop movement    | Immediate   |
| arc       | Draw arc         | Pattern     |
| circle    | Draw arc         | Pattern     |
| reset     | Reset position   | Immediate   |
| clear     | Clear screen     | Immediate   |

> **Note:** Both `arc` and `circle` currently draw only an arc pattern.

---

## Technical Implementation

### ROS 2 Concepts Demonstrated
- Multi-node communication via topics
- Real-time message publishing and subscription
- Service integration with turtlesim
- Parameter management for voice recognition settings
- Modular node architecture for maintainability

### Launch Configuration
The `launch_ninja_turtle.py` file orchestrates the startup of:
- Turtlesim node
- Turtle controller node
- Parameter initialization

---

## Troubleshooting

### Common Issues
- **Microphone access:** Ensure proper audio device permissions
- **Speech recognition accuracy:** Speak clearly with minimal background noise
- **Network dependency:** Use `dummy_voice_listener.py` for offline operation

### Performance Tips
- Use manual commands for precise control during development
- Test voice commands in quiet environments for better recognition
- Utilize the TTS feedback to confirm command recognition

---

This package showcases practical applications of speech processing in robotics, demonstrating how natural language interfaces can enhance human-robot interaction in ROS 2 environments.