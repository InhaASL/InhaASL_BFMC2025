cmake_minimum_required(VERSION 3.0.2)
project(inha_asl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  ackermann_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ackermann_odom_node src/odom_pub.cpp)
target_link_libraries(ackermann_odom_node
  ${catkin_LIBRARIES}
)

add_executable(ekf_odom_node src/ekf_odom.cpp)
target_link_libraries(ekf_odom_node
  ${catkin_LIBRARIES}
)

add_executable(global_pose_tf_broadcaster src/GlobalPoseTFBroadcaster.cpp)
target_link_libraries(global_pose_tf_broadcaster ${catkin_LIBRARIES})

add_executable(global_pose_transformer src/globalTransformer.cpp)
target_link_libraries(global_pose_transformer ${catkin_LIBRARIES})

add_executable(global_pose_publisher src/global_pose_pub.cpp)
target_link_libraries(global_pose_publisher ${catkin_LIBRARIES})

add_executable(global_pose_publisher1 src/global_pose_publisher1.cpp)
target_link_libraries(global_pose_publisher1 ${catkin_LIBRARIES})
