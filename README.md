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

#### 3.1 Setting up Dependencies
- Clone [`unitree_legged_sdk`](https://github.com/unitreerobotics/unitree_legged_sdk) and [`unitree_ros2_to_real`](https://github.com/unitreerobotics/unitree_ros2_to_real) here and build all of them (i.e. clone the SDK into unitree_ros2_to_real and build both SDKs)
```bash
mv unitree_ros2_to_real/ros2_unitree_legged_msgs .
mv Robodog_GPT_Control/robodog_gpt .
mv Robodog_GPT_Control/robodog_rqt .
cd robodog_gpt
mkdir include
cd include
```
- Fix unitree_ros2_to_real by changing `low_udp(LOWLEVEL)` to `low_udp(LOWLEVEL, 8090, "192.168.123.10", 8007)` in `unitree_ros2_to_real/src/ros2_udp.cpp`
- Make sure to change the `unitree_legged_sdk-master` in the cmake file of of unitree_ros2_to_real to `unitree_legged_sdk`

- Clone [`openai-cpp`](https://github.com/olrea/openai-cpp),  [`Whisper.cpp`](https://github.com/ggml-org/whisper.cpp), and [`miniaudio`](https://github.com/mackron/miniaudio)
- Set up Whisper.cpp
```bash
cd whisper.cpp
sh ./models/download-ggml-model.sh small.en # (Recommended to change to medium.en or large.en)
cmake -B build
cmake --build build -j --config Release
```

#### 3.2 Setting up Parameters
- Change `setup.sh` in `robodog_gpt/setup.sh` so that it uses your designated robot's IP, Raspberry Pi's IP, and your OpenAI API key

#### 3.3 Build the packages and launch
```bash
cd ~/ros2_ws
source ./opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source ~/ros2_ws/install/setup.bash
source ./src/robodog_gpt/setup.sh
```
- On one terminal run 
```bash
ros2 run unitree_legged_real ros2_udp highlevel

```
- And on another run
```bash
ros2 launch robodog_gpt robodog_launch.py
```

## 4. Docker Support
- Build with `docker build -t robodog:latest -f src/robodog_gpt/docker/Dockerfile .`
- Run with `docker run -it robodog:latest`
