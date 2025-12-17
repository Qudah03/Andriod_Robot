# ORB Android Robot Controller

This Android application serves as a central control hub for an educational mobile robot powered by the ORB microcontroller. It transforms a smartphone into a powerful sensor and controller, enabling manual teleoperation via Bluetooth and autonomous navigation using computer vision.

## 🚀 Features

### 1. Manual Control (Teleoperation)
- **Virtual Joystick:** Intuitive touch-based joystick for omnidirectional control.
- **Speed Control:** Adjustable speed slider to limit maximum motor output.
- **Differential Drive:** Real-time mixing of X/Y joystick inputs into Left/Right motor signals.

### 2. Autonomous Line Following 
- **Computer Vision:** Uses the phone's camera to detect dark lines on light surfaces.
- **Algorithm:** Implements a Weighted Average (Centroid) algorithm combined with PCA (Principal Component Analysis) for heading calculation.
- **PID Control:** Smooth steering adjustments to keep the robot centered on the path.

### 3. Autonomous Ball Following
- **Object Detection:** Tracks red spherical objects using HSV color space filtering.
- **Blob Detection:** Filters noise and calculates the center of mass and radius of the object.
- **Adaptive Speed:** The robot speeds up when the ball is far away and slows down/stops when close, based on the detected radius.

## 🛠️ Tech Stack

- **Platform:** Android (Min SDK 24, Target SDK 34)
- **Language:** Java
- **Camera:** AndroidX CameraX (ImageAnalysis & Preview)
- **Communication:** Bluetooth Serial (RFCOMM) via HBRS ORB Library
- **UI:** Material Design Components

## 📱 Hardware Requirements

1.  **Android Smartphone:**
    - Must support Bluetooth and have a functioning rear camera.
    - **Mounting:** The phone must be mounted on the robot in **Landscape** orientation, angled approximately 45° downwards facing the floor.
2.  **Robot Chassis:**
    - Differential drive setup (2 DC Motors).
    - ORB Microcontroller (or compatible Bluetooth-enabled controller).

## ⚙️ Setup & Configuration

### Prerequisites
- Android Studio Koala or newer.
- An ORB-compatible robot.

### Installation
1.  Clone this repository.
2.  Open the project in Android Studio.
3.  Build and Run on your Android device.

### Permissions
The app requires the following permissions (requested at runtime):
- `BLUETOOTH_SCAN` & `BLUETOOTH_CONNECT` (Android 12+)
- `CAMERA`

## 🕹️ How to Use

1.  **Connect:**
    - Open the app.
    - Tap **Connect** and select your robot's Bluetooth ID.
    - Wait for the status to change to "Connected".

2.  **Manual Mode:**
    - Ensure no autonomous modes are active.
    - Use the virtual joystick to drive. Adjust the slider to change max speed.

3.  **Autonomous Modes:**
    - **Toggle Camera:** Tap the camera icon/button to enable the preview.
    - **Line Follower:** Tap "Start Line". The robot will autonomously track black lines.
    - **Ball Follower:** Tap "Start Ball". The robot will chase red objects.
    - *Note: Only one mode can be active at a time.*

## 🧩 Architecture Overview

- **MainActivity:** Manages the UI, Bluetooth connection lifecycle, and sensor orchestration. It locks the orientation to Portrait to maintain connection stability while forcing the Camera analysis engine to operate in Landscape mode.
- **LineFollowerAnalyzer:** Analyzes the bottom 50% of the camera frame. It converts the image to grayscale, thresholds it to find dark pixels, and calculates a steering vector.
- **BallFollowerAnalyzer:** Converts the image to HSV color space to robustly detect red hues regardless of lighting conditions. It uses a Proportional Controller for both steering and distance management.

## 🔧 Optimization & Tuning

- **Thresholds:** If the line follower is jittery, adjust the brightness threshold in `LineFollowerAnalyzer.java`.
- **Color Tuning:** If the ball is not detected, adjust the HSV ranges in `BallFollowerAnalyzer.java`.
- **Performance:** The app uses cached byte arrays to minimize Garbage Collection overhead, ensuring smooth real-time performance.

## 📄 License

[Add your license here, e.g., MIT License]

---
*Developed for the HBRS Mobile Robots Course.*
