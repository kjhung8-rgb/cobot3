cmake_minimum_required(VERSION 3.5)
project(cobot3_navigation)

find_package(ament_cmake REQUIRED)

# Install parameter files
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

# Install shared config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
