# 🤖 Robovacuum: Low-Cost Autonomous Vacuum Robot with Visual SLAM & AI-Based Navigation (ROS 2)

Welcome to the development of **Robovacuum** – a real-hardware autonomous vacuum cleaning robot project built using **ROS 2**, aiming to replace traditional LiDAR-based mapping with **low-cost alternatives** like **Visual SLAM**, camera-based localization, and potentially **AI-driven navigation** strategies.

This is an **ongoing university project** with the goal of developing a smart, modular, and cost-effective robotic vacuum cleaner that can intelligently navigate and clean indoor environments using affordable sensors and compute power.

---

## 📌 Project Goals

- Design and implement a **real robot** that can clean autonomously in unknown indoor environments.
- Avoid expensive **LiDAR** sensors by exploring:
  - 📷 Visual SLAM (e.g., ORB-SLAM2, RTAB-Map, etc.)
  - 🧠 AI-based navigation using computer vision or learning-based approaches
- Integrate sensors like:
  - 🧭 IMU (Inertial Measurement Unit)
  - 🌀 Wheel encoders
  - 📏 Ultrasonic sensors
  - 🎮 Joystick (Logitech F710) for manual control
- Perform **sensor fusion** using **Kalman Filters** for better state estimation and odometry.
- Develop and deploy software on **ROS 2 (Humble)** with **Raspberry Pi** and **Arduino**-based hardware.

---

## 📂 Repository Structure

| Package | Description |
|--------|-------------|
| `robovacuum_bringup` | ROS 2 launch files and bringup configuration |
| `robovacuum_controller` | Joystick teleoperation and velocity control |
| `robovacuum_description` | URDF model and robot physical description |
| `robovacuum_firmware` | Arduino firmware for motor control and sensor readings |

---

## 🚧 Current Status (August 2025)

- ✅ ROS 2 workspace and packages initialized
- ✅ Joystick control via Logitech F710 successfully implemented
- ✅ Differential drive odometry with wheel encoders tested
- ✅ Kalman Filter implemented for fusion of encoder + IMU data
- ✅ Basic real hardware prototype assembled and functional
- 🔄 Visual SLAM pipeline under evaluation (ORB-SLAM2, VSLAM, etc.)
- 🔄 AI-based decision making under consideration (ML/CV/RL methods)
- 🔄 Sensor calibration and testing in progress

---

## 🛠 Technologies Used

- **Robot Operating System (ROS 2 Humble)**
- **C++ / Python** programming
- **Kalman Filter** for sensor fusion
- **URDF & RViz** for modeling and simulation
- **Arduino Uno/Nano** for low-level control
- **Raspberry Pi 4** for high-level control
- **Logitech Gamepad F710** for teleoperation

---

## 🔮 Upcoming Milestones

- [ ] Integrate camera module and test visual SLAM algorithms
- [ ] Implement real-time obstacle avoidance using vision or ultrasonic fusion
- [ ] Tune PID control and velocity mapping for smoother motion
- [ ] Develop intelligent cleaning strategy (e.g., coverage planning or learning-based navigation)
- [ ] Final field test and performance evaluation in real-world scenarios

---

## 🧠 Learning Outcomes & Skills Gained

This project offers hands-on experience with:

- 📡 Sensor integration and multi-sensor fusion
- 🧠 Kalman filter design for localization
- 🔁 Differential drive robot control
- 🎮 Joystick interfacing and velocity command mapping
- 🛠 Firmware-hardware-software integration
- 🧩 Real-time robot system architecture on ROS 2
- 📷 Vision and AI in embedded robotics

---

## 📍 Note

> Robovacuum is a **work-in-progress educational project** developed as part of a university curriculum focused on robotics, embedded systems, and AI. The aim is to build a fully autonomous system using real hardware with low-cost components, exploring cutting-edge navigation alternatives beyond LiDAR.

---

## 🤝 Contribution & Collaboration

At the moment, this is a student-led solo project, but future collaboration and code contributions are welcome once the system matures.

---


