# ANA - Autonomous Navigation Assembly

After hundreds of hours of development, going down countless debugging rabbit holes, and a few realizations that the error I've been trying to fix for days was just a simply namespace error, I've completed the first major milestone and goal for this project. In the beginning I simply wanted to apply a few things I learned from my robotics software engineer internship, but that quickly turned into a full-scale personal project. I present ANA, a DIY wheeled robot that uses sensor fusion and SLAM to map out and autonomously navigate complex terrains. ANA uses input from depth imaging, pointcloud, rgb camera, encoder motors, and various other sensors to make inferences and calculated decisions. I know that with enough time I'll look back at ANA as a rather simple project, but to complete it I had to go through development and debugging hell that overall made me a better engineer. 

## Overview
- Linux: Ubuntu 22.04 LTS
- ROS2: Humble
- Software: Gazebo classic, RViz2, RTAB-Map
- Features: SLAM, Nav2, Depth Sensing, Pointcloud Processing, Skid Steer, Odometry/Encoder

## 3D Printed Chassis
- Perception: D435i, Velodyne VLP-16, Livox Mid-360
- Computations: NVIDIA Jetson Orin Nano, Arduino Leonardo
- Other: Arduino REV3 Motor Shield, Quadrature Encoder Motors, Lipo/Battery
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car.jpg" width="60%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_moving.gif" width="25.3%" />
</p>

## RTAB-Map SLAM
- Livox Mid-360
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/1.gif" width="100%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/2.gif" width="100%" />
</p>

- Intel D435i

## Nav2
- A node processes the SLAM-created 3D pointcloud and creates a 2D costmap based on height
- Nav2 performs path planning and autonomous navigation based on the odometry, static global, and live local costmap (Lidar, Depth)
- Controller uses skid-steer
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/3.gif" width="100%" />
</p>

## Prototypes
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_stuff.jpg" width="50%" />
</p>
