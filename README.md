# Talking_Turtle - Voice-Controlled Turtle Movement

A ROS 2 package that enables voice control of turtlesim with speech recognition and text-to-speech feedback enabling the transformation of turtlesim into an interactive, voice-controlled drawing game for kids.

---

## Overview

The `ninja_turtle` package provides an interactive way to control turtle movement in turtlesim using voice commands. The system combines speech recognition with real-time turtle control, offering both voice-activated and manual input modes for comprehensive turtle navigation. 

The project has evolved from a simple voice-controlled drawing tool into an interactive, educational game. By moving to a modular, multi-node ROS 2 architecture, the system is now more scalable and robust. The core of the new experience is the "Draw with Me" mode, where the turtle acts as a collaborative partner, guiding children to draw shapes like squares and rectangles. 

---

## View Demo here [Talking Turtle](https://sh-manu.framer.website/letters/talking-turtle)


## Core Features
- **Interactive "Draw with Me" Game:** A guided mode where the turtle teaches you to draw shapes step-by-step.
- **Voice Recognition:** Real-time speech-to-command conversion using Google Speech API
- **Text-to-Speech Feedback:** Audio confirmation of recognized commands by the node itself
- **Modular Architecture:** The system is divided into distinct nodes for voice input, game logic, speech output, and motor control, making it easy to extend.
- **Multiple Input Methods:** Voice control and manual command input using `command_publisher.py`
- **Offline Speech Recognition:** Alternative implementation for offline use using pyttsx3, sounddevice, and Vosk

---

## System Architecture

```

Voice Input → [voice_listener] → /recognized_speech → [game_manager] → /speak_this → [tts_node] → Spoken Output
                                                                   ↓
                                                             /turtle_cmd
                                                                   ↓
                                                           [turtle_controller] → /turtle1/cmd_vel → [turtlesim]


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

### 3. `tts_node.py` - The Voice of the Turtle
 - Function: Subscribes to the /speak_this topic and uses a text-to-speech engine to convert any message it receives into spoken audio.

### 4. `game_manager.py` - The Brains of the Operation
 - Function: Listens to /recognized_speech and decides what to do. It manages the state of the "Draw with Me" game.
- **Logic:**
- If it hears "let's draw," it starts the game.
- It sends instructions to the user by publishing text to the /speak_this topic.
- It sends movement commands to the turtle by publishing to the /turtle_cmd topic.
- Educational Content: Includes fun facts about shapes to share upon completion of a drawing.

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

## Usage Instructions for drawing node - MAIN FUNCTIONALITY.

### Launch the System
Source your ROS environment. For Mac/robostack users like me:

```bash
pixi shell -e humble
```

Build the workspace:
```bash
pixi run -e humble build
```

In your first terminal, launch the main application:
This command starts the turtlesim, controller, voice listener, and the game manager.
```bash
ros2 launch ninja_turtle launch_draw_with_me.py
```

In a second terminal, launch the voice of the turtle after activating pixi/souce
```bash
ros2 run ninja_turtle tts_node
```
### You should have everything ready to test and play around. This approach makes the system more robust and is a great technical detail to mention when showcasing the project.


alternatively - This files launch all nodes in one single terminal - simplest of all :
```bash
ros2 launch ninja_turtle launch_drawing_ninja.py
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

## Usage Instructions for simple voice control

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
| let's draw| Start "Draw with Me" mode | Guided |

> **Note:** Both `arc` and `circle` currently draw only an arc pattern.
> **Note:** `"let's draw"` starts the collaborative drawing game.

---

## Technical Implementation

### ROS 2 Concepts Demonstrated
- Multi-node communication via topics
- Real-time message publishing and subscription
- Service integration with turtlesim
- Parameter management for voice recognition settings
- Modular node architecture for maintainability
- Educational, interactive gameplay

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