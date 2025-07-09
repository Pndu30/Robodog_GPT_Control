# Robodog GPT Control

This project contains two ROS 2 packages designed to control Unitree GO1 robodog using GPT commands.

---

## 1. Packages Information

### `robodog_gpt`
The core control package responsible for:
- Sending control commands to the robot
- GPT-based natural language command processing (via ROS 2 services)
- Publishing to `/high_cmd`

### `robodog_rqt`
The GUI package that provides a PyQt5 rqt plugin interface to:
- Send commands
- GPT service calls

---

## 2. Dependencies
### Ensure the following dependencies are installed:
- [`unitree_legged_sdk`](https://github.com/unitreerobotics/unitree_legged_sdk)
- [`unitree_ros2_to_real`](https://github.com/unitreerobotics/unitree_ros2_to_real)
- [`openai-cpp`](https://github.com/olrea/openai-cpp)
- `PyQt5` (installed via pip) `pip3 install PyQt5`

## 3. Setup
### Do the following to set up the packages 
- `mkdir /ros2_ws/src`
-  `cd /ros2_ws/src`
- Install/clone all the dependencies here
- `mv unitree_ros2_to_real/ros2_unitree_legged_msgs . `
- `colcon build`
- `source install/setup.bash`
- `ros2 launch robodog_gpt robodog_launch.py`

## 4. Docker Support
Build with `docker build -t robodog:latest -f src/robodog_gpt/docker/Dockerfile .`
Run with `docker run -it robodog:latest`
