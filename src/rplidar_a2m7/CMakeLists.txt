cmake_minimum_required(VERSION 3.8)
project(rplidar_a2m7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)
include_directories(include/rplidar_a2m7/sdk/include)
include_directories(include/rplidar_a2m7/sdk/src)
include_directories(include/rplidar_a2m7/sdk/src/arch/linux)
include_directories(include/rplidar_a2m7/sdk/src/hal)

# Create executable
add_executable(${PROJECT_NAME}_node
  src/rplidar_a2m7_node.cpp
  include/rplidar_a2m7/sdk/src/rplidar_driver.cpp
  include/rplidar_a2m7/sdk/src/hal/thread.cpp
  include/rplidar_a2m7/sdk/src/arch/linux/net_serial.cpp
  include/rplidar_a2m7/sdk/src/arch/linux/net_socket.cpp
  include/rplidar_a2m7/sdk/src/arch/linux/timer.cpp
)

# Link executable
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
)

# Install
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Scripts folder removed as per requirements

# Install rviz files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install include files
install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()
