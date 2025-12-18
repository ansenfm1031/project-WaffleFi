QT += core gui widgets
CONFIG += c++17

TEMPLATE = app
TARGET = wifi_qt

ROS_ROOT = /opt/ros/humble

INCLUDEPATH += $$ROS_ROOT/include
INCLUDEPATH += $$ROS_ROOT/include/rclcpp_action
INCLUDEPATH += $$ROS_ROOT/include/rcl_action
INCLUDEPATH += $$ROS_ROOT/include/action_msgs
INCLUDEPATH += $$ROS_ROOT/include/unique_identifier_msgs
INCLUDEPATH += $$ROS_ROOT/include/nav2_msgs
INCLUDEPATH += $$PWD/3rdparty/QHeatMap/include
SOURCES += $$PWD/3rdparty/QHeatMap/lib/*.cpp
HEADERS += $$PWD/3rdparty/QHeatMap/include/*.h \


INCLUDEPATH += \
    $$ROS_ROOT/include \
    $$ROS_ROOT/include/tf2 \
    $$ROS_ROOT/include/tf2_ros \
    $$ROS_ROOT/include/tf2_msgs \
    $$ROS_ROOT/include/tf2_geometry_msgs \
    $$ROS_ROOT/include/rclcpp \
    $$ROS_ROOT/include/rcl \
    $$ROS_ROOT/include/rmw \
    $$ROS_ROOT/include/rcutils \
    $$ROS_ROOT/include/rcpputils \
    $$ROS_ROOT/include/rclcpp_action \
    $$ROS_ROOT/include/rcl_action \
    $$ROS_ROOT/include/rclcpp_action/rclcpp_action \
    $$ROS_ROOT/include/rcl_yaml_param_parser \
    $$ROS_ROOT/include/tracetools \
    $$ROS_ROOT/include/libstatistics_collector \
    $$ROS_ROOT/include/statistics_msgs \
    $$ROS_ROOT/include/rosidl_typesupport_interface \
    $$ROS_ROOT/include/builtin_interfaces \
    $$ROS_ROOT/include/rcl_interfaces \
    $$ROS_ROOT/include/action_msgs \
    $$ROS_ROOT/include/nav_msgs \
    $$ROS_ROOT/include/std_msgs \
    $$ROS_ROOT/include/msgs \
    $$ROS_ROOT/include/nav2_msgs \
    $$ROS_ROOT/include/geometry_msgs \
    $$ROS_ROOT/include/rosidl_runtime_cpp \
    $$ROS_ROOT/include/rosidl_runtime_c \
    $$ROS_ROOT/include/tf2 \
    $$ROS_ROOT/include/tf2_ros \
    $$ROS_ROOT/include/tf2_geometry_msgs

# 링크 옵션
QMAKE_LFLAGS += -Wl,--no-as-needed
QMAKE_LFLAGS += -Wl,-rpath,$$ROS_ROOT/lib


LIBS += \
    -L$$ROS_ROOT/lib \
    -lrclcpp_action \
    -lrcl_action \
    -lrclcpp \
    -lstatistics_collector \
    -ltf2_ros \
    -ltf2 \
    -lnav_msgs__rosidl_typesupport_cpp \
    -lgeometry_msgs__rosidl_typesupport_cpp \
    -lstatistics_msgs__rosidl_typesupport_cpp \
    -lstatistics_msgs__rosidl_typesupport_c \
    -lnav2_msgs__rosidl_typesupport_cpp \
    -lstatistics_msgs__rosidl_typesupport_introspection_cpp \
    -lrmw_fastrtps_cpp \
    -lrmw_fastrtps_shared_cpp \
    -lrosidl_typesupport_fastrtps_cpp \
    -lrosidl_typesupport_fastrtps_c \
    -lrosidl_typesupport_cpp \
    -lrosidl_typesupport_c \
    -lrosidl_typesupport_introspection_cpp \
    -lrosidl_typesupport_introspection_c \
    -lrosidl_runtime_c \
    -lrcl \
    -lrcl_yaml_param_parser \
    -lrcpputils \
    -lrcutils \
    -ltracetools \
    -lament_index_cpp \
    -lrmw

LIBS += \
    -ltf2 \
    -ltf2_ros

LIBS += \
    -lgeometry_msgs__rosidl_typesupport_cpp \
    -lnav_msgs__rosidl_typesupport_cpp

LIBS += \
    -lrclcpp_action \
    -lnav2_msgs__rosidl_typesupport_cpp \
    -laction_msgs__rosidl_typesupport_cpp

# LIBS += \
#     -L$$ROS_ROOT/lib \
#     -lrclcpp \
#     -lrcl \
#     -ltf2_ros \
#     -ltf2_geometry_msgs \
#     -ltf2 \
#     -ltf2_msgs \
#     -lgeometry_msgs__rosidl_typesupport_cpp \
#     -lstatistics_collector \
#     -lrcutils \
#     -lrmw \
#     -lrcpputils \
#     -ltracetools \
#     -lament_index_cpp \
#     -lrcl_yaml_param_parser \
#     -lrosidl_runtime_c \
#     -lrosidl_typesupport_c \
#     -lrosidl_typesupport_cpp \
#     -lrosidl_typesupport_introspection_c \
#     -lrosidl_typesupport_introspection_cpp \
#     -lrmw_fastrtps_cpp \
#     -lrmw_fastrtps_shared_cpp \
#     -lrosidl_typesupport_fastrtps_c \
#     -lrosidl_typesupport_fastrtps_cpp \
#     -lnav_msgs__rosidl_typesupport_cpp \
#     -lstatistics_msgs__rosidl_typesupport_c \
#     -lstatistics_msgs__rosidl_typesupport_cpp \
#     -lstatistics_msgs__rosidl_typesupport_introspection_cpp


SOURCES += \
    main.cpp \
    mainwindow.cpp \
    rosworker.cpp

HEADERS += \
    mainwindow.h \
    rosworker.h

FORMS += \
    mainwindow.ui



