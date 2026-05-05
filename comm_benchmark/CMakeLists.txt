cmake_minimum_required(VERSION 3.16)
project(comm_benchmark VERSION 0.1.0)

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
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- generated message ----
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Payload.msg
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(comm_bench_msgs_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- core static lib ----
add_library(bench_core STATIC
  src/cli.cpp
  src/stats.cpp
  src/bench_run.cpp
  src/transports/raw_udp.cpp
  src/transports/ros2_be.cpp
  src/transports/ros2_mte.cpp
)
target_include_directories(bench_core PUBLIC include)
ament_target_dependencies(bench_core rclcpp std_msgs)
target_link_libraries(bench_core
  ${comm_bench_msgs_target}
  pthread
  rt
)

# ---- executables ----
add_executable(bench_a src/bench_a_main.cpp)
target_link_libraries(bench_a bench_core)
ament_target_dependencies(bench_a rclcpp)

add_executable(bench_b src/bench_b_main.cpp)
target_link_libraries(bench_b bench_core)
ament_target_dependencies(bench_b rclcpp)

install(TARGETS bench_a bench_b
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
