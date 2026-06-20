cmake_minimum_required(VERSION 3.8)
project(oa_mit)

# oa_mit — bilateral force-feedback teleop on the openarmx MIT layer.
# The bilateral node is DERIVED from openarmx_teleop_bimanual
# (teleop_bimanual_with_gravitycomp_single.cpp), CC-BY-NC-SA-4.0; the KDL
# dynamics (src/dynamics.cpp, include/dynamics.hpp) are copied verbatim from
# the same upstream with attribution. Added: follower /joint_states coupling
# for force reflection.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(openarmx_can REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

# KDL dynamics (gravity) — copied from openarmx_teleop_bimanual
add_library(oa_mit_dynamics STATIC src/dynamics.cpp)
target_include_directories(oa_mit_dynamics PUBLIC include)
ament_target_dependencies(oa_mit_dynamics
  urdf orocos_kdl kdl_parser urdfdom urdfdom_headers)
target_link_libraries(oa_mit_dynamics openarmx_can::openarmx_can)

# Bilateral force-feedback leader node
add_executable(oa_mit_bilateral_node src/oa_mit_bilateral_node.cpp)
target_link_libraries(oa_mit_bilateral_node oa_mit_dynamics)
ament_target_dependencies(oa_mit_bilateral_node
  rclcpp std_msgs sensor_msgs)

install(TARGETS oa_mit_bilateral_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
