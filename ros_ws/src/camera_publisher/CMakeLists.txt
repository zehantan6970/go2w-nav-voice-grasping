cmake_minimum_required(VERSION 3.8)
project(camera_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- 关键修复：添加 ddscxx 路径 ---
include_directories(
  include
  /home/unitree/unitree_sdk2/include
  /home/unitree/unitree_sdk2/thirdparty/include
  /home/unitree/unitree_sdk2/thirdparty/include/ddscxx
)

add_executable(video_publisher src/video_publisher.cpp)

ament_target_dependencies(video_publisher rclcpp sensor_msgs)

# 链接库 (保持不变)
target_link_libraries(video_publisher
  /home/unitree/unitree_sdk2/lib/aarch64/libunitree_sdk2.a
  /home/unitree/unitree_sdk2/thirdparty/lib/aarch64/libddsc.so
  /home/unitree/unitree_sdk2/thirdparty/lib/aarch64/libddscxx.so
)

install(TARGETS video_publisher DESTINATION lib/${PROJECT_NAME})
ament_package()
