cmake_minimum_required(VERSION 3.5)
project(odom_from_imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(/opt/ros/foxy/include/
   ../serial-ros2/include)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
# find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/odom_from_imu_node.cpp)


ament_target_dependencies(${PROJECT_NAME} 
 
 geometry_msgs 
#  tf2_ros
 tf2_sensor_msgs
 serial
 tf2_geometry_msgs
 nav_msgs 
 rclcpp 
 sensor_msgs)


install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
