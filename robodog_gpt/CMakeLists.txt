cmake_minimum_required(VERSION 3.8)
project(robodog_gpt)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

# Service definition
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Comms.srv"
  "srv/Audio.srv"
  DEPENDENCIES builtin_interfaces
)

# Executables
add_executable(control src/control.cpp)
target_include_directories(control PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_ros2_to_real/include
)

add_executable(audio src/audio.cpp
    src/audio.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/miniaudio/miniaudio.c
)
target_include_directories(audio PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/miniaudio
  ${CMAKE_CURRENT_SOURCE_DIR}/include/whisper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_ros2_to_real/include
)

add_executable(gpt_node src/gpt_node.cpp)
target_include_directories(gpt_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/openai-cpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_legged_sdk/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../unitree_ros2_to_real/include
)

ament_target_dependencies(control
  rclcpp
  geometry_msgs
  ros2_unitree_legged_msgs
  rosidl_default_runtime
)

ament_target_dependencies(gpt_node
  rclcpp
  ros2_unitree_legged_msgs
  rosidl_default_runtime
)

ament_target_dependencies(audio
  rclcpp
  ros2_unitree_legged_msgs
  rosidl_default_runtime
)

# Typesupport and linking
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(control
  ${cpp_typesupport_target})

target_link_libraries(gpt_node
  ${cpp_typesupport_target}
  nlohmann_json::nlohmann_json
  CURL::libcurl
)

target_link_libraries(audio
  ${cpp_typesupport_target}
)

# Install and export
install(TARGETS
  control
  gpt_node
  audio
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
