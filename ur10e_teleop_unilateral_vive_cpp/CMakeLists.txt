cmake_minimum_required(VERSION 3.16)
project(ur10e_teleop_unilateral_vive_cpp VERSION 1.0.0)

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

# OpenVR — required by the Vive leader. libopenvr-dev provides
# /usr/include/openvr/openvr.h and /usr/lib/x86_64-linux-gnu/libopenvr_api.so
find_path(OPENVR_INCLUDE_DIR openvr.h
  PATHS /usr/include/openvr /usr/include /usr/local/include/openvr
  NO_DEFAULT_PATH)
find_library(OPENVR_LIBRARY NAMES openvr_api
  PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)
if(NOT OPENVR_INCLUDE_DIR OR NOT OPENVR_LIBRARY)
  message(FATAL_ERROR
    "OpenVR not found. Install with: sudo apt install libopenvr-dev")
endif()
message(STATUS "OpenVR include: ${OPENVR_INCLUDE_DIR}")
message(STATUS "OpenVR library: ${OPENVR_LIBRARY}")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OPENVR_INCLUDE_DIR}
)

# ---- rt_thread static lib (shared between leader/follower) ----
add_library(rt_thread STATIC src/rt_thread.cpp)
target_include_directories(rt_thread PUBLIC include)
target_link_libraries(rt_thread PUBLIC pthread rt)

# ---- Vive Leader executable ----
add_executable(vive_leader_node
  src/vive_leader_main.cpp
  src/vive_leader_node.cpp
  src/vive_tracker.cpp
  src/calibration.cpp
  src/ur_ik.cpp
  src/ur_jacobian.cpp
  src/config.cpp
)
ament_target_dependencies(vive_leader_node
  rclcpp sensor_msgs std_msgs
)
target_link_libraries(vive_leader_node
  rt_thread
  yaml-cpp
  ${OPENVR_LIBRARY}
)

# ---- Follower executable (unchanged from the UR3e unilateral package) ----
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

install(TARGETS vive_leader_node follower_node jitter_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch resources
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
