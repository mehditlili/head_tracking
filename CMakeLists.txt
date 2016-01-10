cmake_minimum_required(VERSION 2.8.3)
project(head_tracking)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
)
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})

catkin_package(
)
add_executable(head_tracking src/main.cpp)
add_library(tracker src/tracker.cpp)
target_link_libraries(tracker ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(head_tracking ${PCL_LIBRARIES} ${catkin_LIBRARIES} tracker)

