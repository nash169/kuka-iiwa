cmake_minimum_required(VERSION 2.8.3)
project(iiwa_gazebo)

# Find catkin components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_ros_control
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# Gazebo cxx flags should have all the required C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_gazebo
 CATKIN_DEPENDS roscpp gazebo_ros gazebo_ros_control
#  DEPENDS system_lib
)