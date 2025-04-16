cmake_minimum_required(VERSION 3.0.2)
project(inha_asl)

## 컴파일 시 C++11 사용
add_compile_options(-std=c++11)

## 의존 패키지
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
  ackermann_msgs
)

## 메시지 및 서비스 등 포함된 경우 catkin_package에 포함시켜야 하나,
## 현재는 필요하지 않사옵니다.
catkin_package()

## include 디렉토리 지정 (필요 시)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 실행 파일 정의
add_executable(ackermann_odom_node src/odom_pub.cpp)

## 링커에 필요한 라이브러리 연결
target_link_libraries(ackermann_odom_node
  ${catkin_LIBRARIES}
)
