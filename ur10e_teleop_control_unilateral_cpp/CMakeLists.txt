cmake_minimum_required(VERSION 3.16)
project(ur10e_teleop_control_unilateral_cpp VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# ---- rt_thread static lib (shared between leader/follower) ----
add_library(rt_thread STATIC src/rt_thread.cpp)
target_include_directories(rt_thread PUBLIC include)
target_link_libraries(rt_thread PUBLIC pthread rt)

# ---- Leader executable ----
add_executable(leader_node
  src/leader_node.cpp
  src/leader_main.cpp
  src/config.cpp
  src/dashboard_client.cpp
)
ament_target_dependencies(leader_node
  rclcpp sensor_msgs std_msgs ur_client_library
)
target_link_libraries(leader_node
  rt_thread
  yaml-cpp
)

# ---- Follower executable ----
add_executable(follower_node
  src/follower_node.cpp
  src/follower_main.cpp
  src/config.cpp
  src/dashboard_client.cpp
  src/ur_jacobian.cpp
)
ament_target_dependencies(follower_node
  rclcpp sensor_msgs std_msgs ur_client_library
)
target_link_libraries(follower_node
  rt_thread
  yaml-cpp
)

# ---- Jitter benchmark (no ROS, no robot; pure timing) ----
add_executable(jitter_benchmark src/jitter_benchmark.cpp)
target_link_libraries(jitter_benchmark rt_thread)

install(TARGETS leader_node follower_node jitter_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch resources
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
