# filepath: /home/mosaic/mosaic/manipulation_ws/src/action/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(action)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_interfaces REQUIRED)
ament_package()