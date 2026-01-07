cmake_minimum_required(VERSION 3.8)
project(wifi_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WifiRssi.msg"
  "msg/WifiFused.msg"
  "srv/GetHeatmap.srv"
  "srv/ListSessions.srv"
  "srv/DeleteSession.srv"
  "srv/ListSsids.srv"
  DEPENDENCIES std_msgs
)

ament_package()
