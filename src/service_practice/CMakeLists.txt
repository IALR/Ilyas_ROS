cmake_minimum_required(VERSION 3.5)
project(service_practice)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Calculator.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
