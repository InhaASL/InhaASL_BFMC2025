cmake_minimum_required(VERSION 3.0.2)
project(your_package_name)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  ackermann_msgs
)

add_message_files(
  FILES
  CarState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  ackermann_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs ackermann_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
