cmake_minimum_required(VERSION 3.8)
project(openarmx_bilateral)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(relay_node src/relay_node.cpp)
ament_target_dependencies(relay_node rclcpp sensor_msgs std_msgs)

# Phase-1 logger: follower cmd-vs-actual CSV for the gravity-comp A/B test
add_executable(log_node src/log_node.cpp)
ament_target_dependencies(log_node rclcpp sensor_msgs std_msgs)

# Phase-3 friction identification: shuttles each joint through a speed list
add_executable(friction_id_node src/friction_id_node.cpp)
ament_target_dependencies(friction_id_node rclcpp sensor_msgs std_msgs)

# Temporary helper: print per-joint angles (both arms, leader+follower)
add_executable(joint_echo_node src/joint_echo_node.cpp)
ament_target_dependencies(joint_echo_node rclcpp sensor_msgs)

# Phase-3 friction compensation: gravity + Fc*tanh(0.1*k*w) -> effort controller
add_executable(friction_comp_node src/friction_comp_node.cpp)
ament_target_dependencies(friction_comp_node rclcpp sensor_msgs std_msgs)

install(TARGETS relay_node log_node friction_id_node joint_echo_node friction_comp_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
