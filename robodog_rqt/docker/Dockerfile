FROM osrf/ros:humble-desktop-full

# Install deps
RUN apt update && apt install -y \\
    python3-colcon-common-extensions \\
    python3-rosdep \\
    && rm -rf /var/lib/apt/lists/* 


# Set workspace
WORKDIR /ros2_ws/src

RUN python3 -m pip install PyQt5
RUN git clone https://github.com/unitreerobotics/unitree_legged_sdk.git
RUN git clone https://github.com/unitreerobotics/unitree_ros2_to_real.git
RUN git clone https://github.com/olrea/openai-cpp.git
RUN mv unitree_ros2_to_real/ros2_unitree_legged_msgs .

COPY ./robodog_gpt ./robodog_gpt
COPY ./robodog_rqt ./robodog_rqt

# Build your code
SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.sh && \\
    rosdep update && rosdep install --from-paths src --ignore-src -r -y && \\
    colcon build

# Default launch command
COPY src/robodog_rqt/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robodog_gpt", "robodog_launch.py"]
