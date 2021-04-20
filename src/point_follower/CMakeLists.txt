cmake_minimum_required(VERSION 3.0.2)
project(point_follower)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
  nav_core
)


catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(robot src/robotNode.cpp)
target_link_libraries(robot ${catkin_LIBRARIES})

add_executable(path_planner src/astar.cpp)
target_link_libraries(path_planner ${catkin_LIBRARIES})

