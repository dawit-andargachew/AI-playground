# Exercise: ROS2 Services - Turtlebot & Burrow

## Overview

This exercise demonstrates **ROS2 service communication** between two nodes:

- **Burrow Node (Client)**: Represents a turtle's burrow that stores apples. It tracks how many apples are currently stored (n) and the burrow's maximum capacity (s). When resources are low (n < s), it requests help from the turtlebot.

- **Turtlebot Node (Server)**: A robot equipped with LIDAR that scans its surroundings to detect apples (represented as spheres in simulation). When the burrow requests help, it counts nearby apples and determines if there are enough to refill the burrow.

**Communication Flow**:
1. Burrow sends request: "I have n apples, my capacity is s"
2. Turtlebot analyzes LIDAR data and detects apples using clustering algorithm
3. Turtlebot responds: True (enough apples found) or False (not enough)
4. Turtlebot publishes apple positions to `/apples` topic

**Scenarios Tested**: The exercise runs 5 test cases covering different situations (n < s with enough/not enough apples, n = s when burrow is full).

---

## Prerequisites

### Required Software
- **ROS2 Jazzy** (Ubuntu 24.04)
- **C++** (C++17 or higher)
- **Python 3**
- **Gazebo** (3D simulation environment)
- **RViz2** (Visualization tool)

### Installation
```bash
# Install Gazebo and simulation packages
sudo apt update
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-gazebo-ros2-control -y

# Install navigation packages (required by exercise environment)
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-navigation2 -y
```

---

## Build
```bash
cd ~/ws_6_assignment
colcon build
source install/setup.bash
```

---

## Run (Open 3 Terminals)

**Terminal 1** - Launch Simulation (Gazebo + RViz2):
```bash
cd ~/ws_6_assignment
source install/setup.bash
ros2 launch ir_launch exercise_4.launch.py
```
*Wait for Gazebo and RViz2 to fully load before proceeding*

**Terminal 2** - Start Turtlebot Service Server:
```bash
cd ~/ws_6_assignment
source install/setup.bash
ros2 run six_ex4 turtlebot_node
```

**Terminal 3** - Start Burrow Service Client:
```bash
cd ~/ws_6_assignment
source install/setup.bash
ros2 run six_ex4 burrow_node
```

---

## What You'll See

- **Gazebo**: Turtlebot robot surrounded by apple objects (spheres)
- **RViz2**: Visualization of robot, LIDAR scans, and detected apples
- **Terminals**: Detailed logs showing all service requests, LIDAR analysis, and responses

The burrow will automatically send 5 requests with different scenarios, and you'll see the complete communication flow in the terminal output.

