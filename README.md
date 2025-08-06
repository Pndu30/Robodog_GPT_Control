# Robodog GPT Control

This project contains two ROS 2 packages designed to control Unitree GO1 robodog using GPT commands.

---

## 1. Packages Information

### 🔹 `robodog_gpt`
Handles:
- Robot control logic
- GPT-based natural language processing via ROS 2 services
- Audio recording and Whisper-based transcription
- Publishes motion commands to `/high_cmd`

### 🔹 `robodog_rqt`
Provides a PyQt5 GUI:
- Button-based control
- Integration with GPT services

---

## 2. Dependencies
### Ensure the following dependencies are installed:

#### ROS-related:
- [`unitree_legged_sdk`](https://github.com/unitreerobotics/unitree_legged_sdk)
- [`unitree_ros2_to_real`](https://github.com/unitreerobotics/unitree_ros2_to_real)

#### Libraries:
- [`openai-cpp`](https://github.com/olrea/openai-cpp)
- [`PyQt5`](https://pypi.org/project/PyQt5/)
- [`Whisper.cpp`](https://github.com/ggml-org/whisper.cpp)
- [`miniaudio`](https://github.com/mackron/miniaudio)
  
## 3. Setup
### Do the following to set up the packages 
```bash
pip3 install PyQt5
mkdir /ros2_ws/src
cd /ros2_ws/src
  ```
- Clone [`unitree_legged_sdk`](https://github.com/unitreerobotics/unitree_legged_sdk) and [`unitree_ros2_to_real`](https://github.com/unitreerobotics/unitree_ros2_to_real) here
- Clone this package here
```bash
mv unitree_ros2_to_real/ros2_unitree_legged_msgs
cd robodog_gpt
mkdir include
cd include
```
- Clone [`openai-cpp`](https://github.com/olrea/openai-cpp),  [`Whisper.cpp`](https://github.com/ggml-org/whisper.cpp), and [`miniaudio`](https://github.com/mackron/miniaudio)
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source ~/ros2_ws/install/setup.bash
ros2 launch robodog_gpt robodog_launch.py
```

## 4. Docker Support
- Build with `docker build -t robodog:latest -f src/robodog_gpt/docker/Dockerfile .`
- Run with `docker run -it robodog:latest`
