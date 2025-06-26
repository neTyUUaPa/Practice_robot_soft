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

# Установка необходимых пакетов
RUN apt update && apt install -y \
    novnc \
    websockify \
    xvfb \
    x11vnc \
    && rm -rf /var/lib/apt/lists/*

# Копирование скрипта запуска
COPY start.sh /start.sh
RUN chmod +x /start.sh

# Порт NoVNC
EXPOSE 6080

# Запуск скрипта
CMD ["/start.sh"]