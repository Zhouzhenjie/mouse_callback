cmake_minimum_required(VERSION 3.10)
project(mouse_callback)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_definitions(-Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pluginlib
  cv_bridge
  image_geometry
  nodelet
  message_generation
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  std_msgs
  pluginlib
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mouse_callback
  src/mouse_callback.cpp
  src/mouse_callback_node.cpp
)

target_link_libraries(mouse_callback
  ${catkin_LIBRARIES}
)

