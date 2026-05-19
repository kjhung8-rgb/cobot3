#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/dev_ws/cobot3/install/setup.bash"

echo "== expected topics =="
ros2 topic list | grep -E '(/spot_0/scan|/spot_0/odom|/spot_0/front_cam/camera_info|/tf|/tf_static)' || true

echo "\n== /spot_0/scan info =="
ros2 topic info /spot_0/scan || true

echo "\n== /spot_0/odom once =="
ros2 topic echo /spot_0/odom --once || true

echo "\n== TF odom -> spot_0/base_link =="
timeout 5 ros2 run tf2_ros tf2_echo odom spot_0/base_link || true

echo "\n== TF spot_0/base_link -> spot_0/lidar_link =="
timeout 5 ros2 run tf2_ros tf2_echo spot_0/base_link spot_0/lidar_link || true

echo "\n== TF spot_0/base_link -> spot_0/front_cam_link =="
timeout 5 ros2 run tf2_ros tf2_echo spot_0/base_link spot_0/front_cam_link || true
