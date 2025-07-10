# 🤖 Delta Robot for Color-Based Pick and Place

This project implements an automated **delta robot** system capable of identifying colored objects on a main conveyor belt and sorting them onto two dedicated conveyors based on color classification. The system integrates precise kinematics, real-time object detection, and multi-axis motion control.

---
## 🎥 Demo Video

![Demo GIF](docs/videos/Delta_Robot_Demo.mp4)

---

## 🎯 Overview

The robot performs the following functions:

- Detects incoming objects on the main conveyor using a **proximity sensor**.
- Uses the **ESP32-CAM** module to capture images and classify object colors (*green* or *blue*).
- Activates the gripper and rotation servos to pick the object.
- Moves the object to the target conveyor belt depending on its color.

---

## 🛠️ Hardware Components

- **Microcontrollers:**
  - ESP32 (main control unit for kinematics and motion)
  - ESP32-CAM (color detection and image processing)

- **Motors:**
  - 3 × NEMA17 stepper motors (delta robot arms)
  - 3 × TMC stepper drivers (smooth and precise motor control)
  - 3 × DC motors (main conveyor and two sorting conveyors)
  - 2 × Servo motors:
    - Gripper actuation
    - End effector rotation

- **Sensors:**
  - Proximity sensor for object presence detection

---

## ⚙️ Software and Functionality

- **Color Detection:**
  - ESP32-CAM captures images and processes them to classify objects as green or blue.

- **Sorting Logic:**
  - Green objects are placed on Conveyor A.
  - Blue objects are placed on Conveyor B.

- **Motion Coordination:**
  - Stepper motors and servos are synchronized to perform pick and place actions without collisions.

---
