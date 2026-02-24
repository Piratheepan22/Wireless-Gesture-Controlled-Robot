# Wireless Gesture-Controlled Robot

## Project Overview
This project presents a **wireless gesture-controlled robotic system** that allows a user to control a mobile robot using **hand gestures instead of physical controllers** such as joysticks or buttons. The system is divided into two main units: a **gesture transmitter unit** and a **robot receiver unit**, which communicate wirelessly in real time.

---

## System Architecture
The system consists of the following two modules:

### 1. Gesture Transmitter Unit
- A **motion/gesture sensor** (such as an accelerometer or IMU) is used to detect hand movements and orientation (tilt along X and Y axes).
- A **microcontroller** reads the sensor data and processes it to identify predefined gestures (forward, backward, left, right, stop).
- The processed gesture commands are transmitted wirelessly using a **wireless communication module** (e.g., RF, Bluetooth, or similar).

### 2. Robot Receiver Unit
- A second **microcontroller** receives the wireless gesture commands.
- Based on the received command, the microcontroller controls the robot’s motors through a **motor driver circuit** (e.g., L293D / L298N).
- The robot moves in the corresponding direction in **real time**, ensuring low latency and smooth control.

---

## Working Principle
1. The user performs a hand gesture.
2. The gesture sensor converts hand motion into electrical signals.
3. The transmitter microcontroller processes sensor values and maps them to motion commands.
4. The command is sent wirelessly to the robot.
5. The receiver microcontroller decodes the command.
6. Motor driver circuitry drives the motors accordingly, resulting in robot movement.

---

## Key Technologies Used
- **Embedded Systems Programming (C / Arduino / Embedded C)**
- **Gesture sensing using accelerometer / IMU**
- **Wireless communication**
- **Motor control using H-bridge motor driver**
- **Real-time command processing**

---

## Features
- Fully **wireless control**
- **Real-time response** with minimal delay
- Intuitive and natural **human–robot interaction**
- Modular design for easy upgrades and extensions

---

## Applications
- Human–Robot Interaction (HRI)
- Assistive robotics
- Educational robotics projects
- Gesture-based control systems
- Research and prototyping in embedded systems

