cmake_minimum_required(VERSION 3.16)
project(ur10e_teleop_real_py VERSION 1.0.0)

install(PROGRAMS
  src/leader_real_node.py
  src/follower_real_node.py
  script/install.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_package()
endif()
