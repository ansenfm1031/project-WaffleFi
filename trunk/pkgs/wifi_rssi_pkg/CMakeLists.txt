cmake_minimum_required(VERSION 3.8)
project(wifi_rssi_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(wifi_interface REQUIRED)

add_executable(wifi_rssi_node
  src/wifi_rssi_node.cpp
)

ament_target_dependencies(wifi_rssi_node
  rclcpp
  wifi_interface
)

install(TARGETS
  wifi_rssi_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
