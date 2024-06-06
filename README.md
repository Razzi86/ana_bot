# ANA - Autonomous Navigation Assembly

ANA is a person project that I'm very proud of. ANA runs on the ROS2 Humble framework, and uses SLAM, Nav2, 3D Lidar, Depth Images, and various other sensors to autonomous navigate and map complex terrains. This github is meant to showcase ANA's lifspan, and I hope those who view it can be inspired by or learn from my progress and the open source code.

## Hardware

## 3D Printed Chassis
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car.jpg" width="55%" />
</p>

## SLAM
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/1.gif" width="49%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/2.gif" width="49%" />
</p>

## Gallery
- ANA navigating through an obstacle course using its SLAM and Nav2 capabilities.
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/3.gif" width="49%" />
</p>
- Real-time mapping demonstration using the onboard depth camera.

## Framework
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Classic for simulation

## Components
1. **ROS2 Humble**: The core framework providing tools and libraries designed for a robust robotic system.
2. **Gazebo Classic**: For simulation of the robot in a high-fidelity 3D environment.
3. **Depth Camera**: For environmental perception and obstacle detection.
4. **Jetson Orin**: Powers the processing needs for real-time data handling and computation.

## Features
- **SLAM**: Implements RTAB-Map for efficient 3D mapping and localization.
- **Navigation and Path Planning**: Utilizes the Nav2 stack for dynamic path planning and obstacle avoidance.
- **Depth Sensing**: Equipped with a depth camera to perceive and interact with the environment in three dimensions.
- **Skid Steering**: Allows the robot to maneuver tightly and swiftly with a four-wheeled skid steering system.

## 3D printed chassis and assembled components
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_stuff.jpg" width="28%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_moving.gif" width="21%" />
</p>

## RViz2 & Gazebo (new more-in-sync RViz/Gazebo gif to be added soon)
<img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_gif.gif" width="80%" />
