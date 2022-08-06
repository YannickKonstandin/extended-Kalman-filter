cmake_minimum_required(VERSION 3.0.2)
project(konstandin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES konstandin
#  CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs geometry_msgs message_generation nav_msgs tf 
#  DEPENDS system_lib
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_robot src/move.cpp)
target_link_libraries(move_robot ${catkin_LIBRARIES})

add_executable(EKF src/ekf.cpp)
target_link_libraries(EKF ${catkin_LIBRARIES})



