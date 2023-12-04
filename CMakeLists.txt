cmake_minimum_required(VERSION 3.0.2)
project(tokuron_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
)

find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigation src/navigation.cpp)

target_link_libraries(navigation
  ${catkin_LIBRARIES}
  yaml-cpp
)
