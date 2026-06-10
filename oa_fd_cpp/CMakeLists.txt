cmake_minimum_required(VERSION 3.16)
project(oa_fd_cpp VERSION 0.1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

# openarmx CAN library (installed system-wide by openarmx-can_1.0.0 .deb).
find_package(openarmx_can REQUIRED)

include_directories(include)

add_executable(oa_fd_node
  src/main.cpp
  src/teleop_node.cpp
  src/oax_driver.cpp
  src/gravity.cpp
  src/config.cpp
  src/rt_thread.cpp
)
ament_target_dependencies(oa_fd_node
  rclcpp sensor_msgs std_msgs orocos_kdl_vendor kdl_parser urdf
)
target_link_libraries(oa_fd_node
  openarmx_can::openarmx_can
  yaml-cpp
  pthread rt
)

install(TARGETS oa_fd_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME}/)

ament_package()
