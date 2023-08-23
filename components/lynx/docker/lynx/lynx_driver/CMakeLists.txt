cmake_minimum_required(VERSION 2.8.3)
project(lynx_driver)

add_compile_options(-std=c++11)
add_compile_options(-Wno-address-of-packed-member)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  angles
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  dynamic_reconfigure
  lynx_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Pid.cfg
  cfg/Drive.cfg
  cfg/Imu.cfg
)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lynx_driver_node 
  src/lynx_driver_node.cpp
  src/LynxSerialDriver.cpp
  src/modules/ImuModule.cpp
  src/modules/DriveModule.cpp
)

add_executable(lynx_sensors_state_monitor_node 
  src/lynx_state_monitor_node.cpp
)

add_dependencies(lynx_driver_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

add_dependencies(lynx_sensors_state_monitor_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(lynx_driver_node
  ${catkin_LIBRARIES}
)

target_link_libraries(lynx_sensors_state_monitor_node
  ${catkin_LIBRARIES}
)

add_executable(lynx_teleop_node 
  src/tools/lynx_teleop_node.cpp
)

add_dependencies(lynx_teleop_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(lynx_teleop_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS lynx_driver_node lynx_teleop_node lynx_sensors_state_monitor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)