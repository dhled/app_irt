cmake_minimum_required(VERSION 2.8.3)
project(app_irtjv)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  roscpp
  std_msgs
  gazebo_ros
)

catkin_package(
)
