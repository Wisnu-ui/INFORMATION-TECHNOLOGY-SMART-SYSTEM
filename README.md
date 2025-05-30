[Link Karya](https://drive.google.com/drive/folders/1FNbPrl3ttWD-k8DK7k8nIZBa4yzaIEsb)

# Robot Hand Gesture Control System

This project implements a real-time hand gesture control system that allows you to control a robot using hand movements captured through a webcam. The system uses computer vision to track hand gestures and publishes the coordinates and grip state to an MQTT broker.

## Features

- Real-time hand tracking using MediaPipe
- Kalman filter for smooth hand position tracking
- MQTT communication for robot control
- Hand gesture recognition (grip/release)
- Visual feedback with OpenCV
- Coordinate mapping from camera space to robot space

## Prerequisites

- Python 3.x
- Webcam

## Dependencies

```bash
opencv-python
numpy
mediapipe
paho-mqtt
```

## Installation

1. Clone this repository
2. Install the required dependencies:
   ```bash
   pip install opencv-python numpy mediapipe paho-mqtt
   ```

## Configuration

The system uses the following default settings:

- MQTT Broker: HiveMQ Cloud
- Camera Resolution: 1280x720
- Frame Rate: 120 FPS
- Robot Control Area: 
  - X: 250-950 pixels (maps to robot E-axis: 330-0)
  - Y: 250-650 pixels (maps to robot Z-axis: 200 to -100)

## Usage

1. Run the main script:
   ```bash
   python DeteksiGesturPublish.py
   ```

2. Position your right hand within the white rectangle (Robot Control Area) shown on the screen.

3. The system will track your hand movements:
   - Hand position is mapped to robot coordinates (E and Z axes)
   - Closing your hand triggers the "Gripping" state
   - Opening your hand triggers the "Opening" state

4. The system publishes the following data to the MQTT topic "XY":
   - Robot E-axis position
   - Robot Z-axis position
   - Hand state (0 for open, 1 for closed)
   - Timestamp

5. Press 'q' to quit the application

## Visual Feedback

The application window shows:
- Hand tracking landmarks
- Current hand position
- Mapped robot coordinates
- Hand state (Gripping/Opening)
- Robot control area boundaries

## MQTT Message Format

Messages are published in the following format:
```
E,Z,hand_state,timestamp
```
Where:
- E: Robot E-axis position (0-330)
- Z: Robot Z-axis position (-100 to 200)
- hand_state: 1 for closed hand, 0 for open hand
- timestamp: Current time in seconds

## Notes

- The system is optimized for right-hand tracking
- Hand position updates are sent every 300ms to prevent overwhelming the robot
- Kalman filtering is used to reduce hand position jitter
- The system requires a stable internet connection for MQTT communication 
