cmake_minimum_required(VERSION 3.5)
project(reactive_gap_follower)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

add_executable(cass_gap_follower src/cass_gap_follower.cpp)

ament_target_dependencies(cass_gap_follower
  "rclcpp"
  "sensor_msgs"
  "ackermann_msgs"
  "geometry_msgs"
  "nav_msgs"
)

install(TARGETS
  cass_gap_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
