# Базовый образ ROS 2 Humble
FROM arm64v8/ros:humble

# Установка RViz2
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Установка дополнительных пакетов (если нужно)
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Команда по умолчанию: запуск RViz2
CMD ["rviz2"]
