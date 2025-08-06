Robovacuum – Real-Time AI-Based Autonomous Vacuum Robot without LiDAR
🚀 Project Overview
This project aims to develop a real-world autonomous vacuum cleaning robot using ROS 2 that avoids the use of expensive LiDAR-based mapping, instead exploring alternatives like Visual SLAM or AI-based navigation strategies. The system is being implemented and tested on real hardware, not just in simulation.

🎯 Project Objectives
Build an autonomous vacuum robot platform that:

Navigates in unknown environments without LiDAR

Uses camera-based or AI-based mapping and localization (Visual SLAM / alternatives under evaluation)

Integrates multiple low-cost sensors for data fusion (e.g., encoders, IMU, ultrasonic)

Can be teleoperated using a joystick

Enable AI-enhanced features for:

Obstacle avoidance

Path optimization

Dynamic decision-making in cluttered environments

Deploy the complete stack on embedded hardware like Raspberry Pi and Arduino

🔧 Technologies & Tools Used
ROS 2 (Humble) for robot software infrastructure

C++ and Python for development

Kalman Filter for sensor fusion

Logitech F710 Gamepad for teleoperation

Differential Drive Odometry implementation

Encoder + IMU + Ultrasonic Sensors integration

Arduino (firmware) and Raspberry Pi (compute unit)

✅ Current Progress
✅ ROS 2 workspace structured into:

robovacuum_bringup: Launch files and system bringup

robovacuum_controller: Joystick control node and velocity mapping

robovacuum_description: URDF and robot model

robovacuum_firmware: Arduino firmware for motor control and sensor reading

✅ Joystick control successfully integrated and tested

✅ Odometry implemented using differential drive and encoder data

✅ Sensor fusion using Kalman Filter implemented for improved localization

✅ Real robot hardware partially assembled and tested in lab

🚧 Ongoing & Future Work
🔄 Final decision on mapping strategy: evaluating Visual SLAM (ORB-SLAM2, VSLAM, RTAB-Map) or AI-based navigation (e.g., reinforcement learning)

🔄 Camera integration and testing of visual features

🔄 AI feature development: real-time obstacle detection and adaptive cleaning logic

🔄 Complete system integration and testing in real-world environment

🔄 Create map-saving and localization-reuse features

💡 Key Learnings (from student’s resume)
Learned and applied ROS 2 control, odometry, and sensor fusion techniques

Hands-on experience in real robot development using low-cost hardware

Exposure to Kalman filtering, embedded firmware, and multi-sensor integration

Practical knowledge in differential drive kinematics, joystick-based control, and robot localization

📌 Note
This is an ongoing university project, currently under active development. The final robot will aim to demonstrate a cost-effective, intelligent, and autonomous cleaning solution leveraging ROS 2 and AI without using LiDAR.
