# ANA - Autonomous Navigation Assembly

After months of development, countless debugging rabbit holes, and the few times when an error I tried to fix for days was simply an incorrect frame_id, I've completed my first major goal for ANA - autonomous navigation. In the beginning I simply wanted to apply a few things I learned from my robotics software engineer internship, but that quickly turned into a full-scale personal project. I present ANA, a DIY wheeled robot that uses sensor fusion and SLAM to map out and autonomously navigate complex terrains. ANA uses input from depth imaging, pointcloud, rgb camera, encoder motors, and various other sensors to make inferences and calculated decisions. I know that with enough time I'll look back at ANA as a rather simple project, but to complete it I had to learn a lot from scratch and go through development/debugging hell, which overall made me a better engineer. 

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
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/VelodyneLivox.jpeg" width="34%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car.jpg" width="60%" />
</p>

## RTAB-Map SLAM
- Livox Mid-360
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/1.gif" width="100%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/2.gif" width="100%" />
</p>

- Intel D435i

## Nav2
- A node processes the SLAM-created 3D pointcloud and creates a 2D global costmap based on height
- The local costmap is created live based on height, and will be used for obstacle avoidance
- Nav2 performs control, path planning, and autonomous navigation based on the odometry, static global, and live local costmap (Lidar, Depth)
- Controller uses skid-steer
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/3.gif" width="74%" />
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_moving.gif" width="24%" />
</p>

## Prototypes
<p float="left">
  <img src="https://github.com/Razzi86/ana_bot/blob/main/src/ana/github_content/car_stuff.jpg" width="50%" />
</p>
