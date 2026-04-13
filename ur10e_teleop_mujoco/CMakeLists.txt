cmake_minimum_required(VERSION 3.16)
project(ur10e_teleop_mujoco VERSION 1.0.0)

install(PROGRAMS
  script/exec_bilateral_sim.sh
  script/sim_install.sh
  src/leader_sim_node.py
  src/follower_sim_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
)

install(DIRECTORY xml/
  DESTINATION share/${PROJECT_NAME}/xml
)

install(DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_package()
endif()
