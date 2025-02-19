# Gazebo ROS Turret Control Plugin

[English](#english) | [Русский](#russian)

## English

### Overview
A ROS 2 Gazebo plugin for precise turret rotation control with multiple acceleration profiles and configurable parameters. The plugin provides both topic-based and action-based interfaces for controlling turret rotation in simulation.

### Features
- Multiple acceleration profiles:
  - Quintic polynomial
  - Cubic polynomial
  - S-curve
  - Trapezoidal velocity
- Configurable parameters:
  - Trajectory duration
  - Maximum acceleration/velocity
  - Encoder resolution
  - Hold time
  - Rotation direction (CW/CCW)
- Position feedback with encoder simulation
- Automatic return to home position
- ROS 2 Action Server interface
- Topic-based control interface

### Dependencies
- ROS 2 (Humble/Iron)
- Gazebo
- gazebo_ros
- rclcpp
- rclcpp_action
- std_msgs
- geometry_msgs

### Installation (read install.xml)