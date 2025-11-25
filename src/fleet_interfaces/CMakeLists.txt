cmake_minimum_required(VERSION 3.8)
project(fleet_interfaces)

# Compiler options for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# FIND DEPENDENCIES
# ============================================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Interface generation tools
find_package(std_msgs REQUIRED)                   # We use Header
find_package(geometry_msgs REQUIRED)              # We use Pose, Point, Twist
find_package(builtin_interfaces REQUIRED)         # We use Time

# ============================================
# GENERATE INTERFACES
# ============================================
# This tells ROS2 to create Python/C++ code from our .msg and .srv files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/FleetCommand.msg"
  "srv/AssignTask.srv"
  "srv/GetFleetStatus.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Required for ament
ament_package()