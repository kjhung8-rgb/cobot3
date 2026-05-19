cmake_minimum_required(VERSION 3.5)
project(cobot3_navigation)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install parameter files
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

# Install map files
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install RViz configs
install(DIRECTORY rviz2/
  DESTINATION share/${PROJECT_NAME}/rviz2
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/cmd_vel_relay.py
  scripts/anymal_cmd_vel_relay.py
  scripts/map_odom_bootstrap.py
  scripts/publish_isaac_spawn_pose.py
  scripts/publish_anymal_spawn_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
