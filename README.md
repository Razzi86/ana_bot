# ANA - Autonomous Navigation Assembly

## Overview
ANA (Autonomous Navigation Assembly) is a personal project that I'm very proud of. ANA is run on the ROS2 framework, using SLAM, Nav2, and 3D Lidar to autonomous navigate complex terrains. While most of ANA's development happens entirely within a simulation, I've also used CAD to prototype multiple (working) robot car chassis - which I hope to finish up soon. This github is meant to showcase ANA in the present and future, and I hope that those who view this github can learn from the open source code I've provided.

<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/SLAM_Live.gif" width="48%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/gazebo_world.jpg" width="28%" />
</p>

## Features
- **Real-Time SLAM**: Implements RTAB-Map for efficient 3D mapping and localization.
- **Navigation and Path Planning**: Utilizes the Nav2 stack for dynamic path planning and obstacle avoidance.
- **Depth Sensing**: Equipped with a depth camera to perceive and interact with the environment in three dimensions.
- **Skid Steering**: Allows the robot to maneuver tightly and swiftly with a four-wheeled skid steering system.

## Components
1. **ROS2 Humble**: The core framework providing tools and libraries designed for a robust robotic system.
2. **Gazebo Classic**: For simulation of the robot in a high-fidelity 3D environment.
3. **Depth Camera**: For environmental perception and obstacle detection.
4. **Jetson Orin**: Powers the processing needs for real-time data handling and computation.

## Framework
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Classic for simulation

## Gallery
- ANA navigating through an obstacle course using its SLAM and Nav2 capabilities.
- Real-time mapping demonstration using the onboard depth camera.

## 3D printed chassis and assembled components
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car.jpg" width="48%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_stuff.jpg" width="28%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_moving.gif" width="21%" />
</p>

## RViz2 & Gazebo (new more-in-sync RViz/Gazebo gif to be added soon)
<img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_gif.gif" width="80%" />
