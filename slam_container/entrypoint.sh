#!/bin/bash

# Логирование
echo "Starting SLAM container..."

# Запуск SLAM
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch slam_package slam_launch.py
