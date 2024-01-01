cmake_minimum_required(VERSION 3.0.2)
project(microstrain_inertial_listener)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mastering_ros_demo_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(microstrain_inertial_listener src/listener.cpp)

target_link_libraries(microstrain_inertial_listener ${catkin_LIBRARIES})
