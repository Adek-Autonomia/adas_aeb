cmake_minimum_required(VERSION 3.0.2)
project(adas_aeb)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_reconfigure
  pcl_conversions
  message_filters
  pcl_ros
  sensor_msgs
  vision_msgs
  visualization_msgs
  tf
  laser_geometry
  roslaunch
  rostest
)
find_package(OpenCV REQUIRED)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/PCL_AEB_.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES adas_aeb
  CATKIN_DEPENDS  roscpp
  DEPENDS OpenCV
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(tof_aeb_node src/tof_aeb/main.cpp 
  src/tof_aeb/setup_port.cpp 
  src/tof_aeb/tokenizer.cpp 
  src/tof_aeb/pipeline.cpp 
  src/tof_aeb/tof_node.cpp
)

add_executable(pcl_aeb_node src/pcl_aeb/simple_aeb_node.cpp
  src/pcl_aeb/cloud_filter.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(tof_aeb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(pcl_aeb_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
  ${OpenCV_LIBRARIES}
)
## Specify libraries to link a library or executable target against
target_link_libraries(tof_aeb_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pcl_aeb_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  roslaunch_add_file_check(launch)
  add_rostest(test/tof.test)
endif()

