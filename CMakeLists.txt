cmake_minimum_required(VERSION 2.8.3)
project(radar_parser)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11 -O3 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Occupancy Mapping
add_executable(${PROJECT_NAME}_publisher src/publisher.cpp)
target_link_libraries(${PROJECT_NAME}_publisher ${catkin_LIBRARIES})