# Базовый образ ROS 2 Humble
FROM arm64v8/ros:humble

    
# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    cmake \
    git \
    build-essential \
    libboost-all-dev \
    libyaml-cpp-dev \
    libeigen3-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Установка ROS 2 пакетов
RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav-msgs \
    ros-humble-tf2 \
    ros-humble-tf-transformations \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav2-msgs \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*
# Копирование твоего SLAM-пакета
COPY src/ /workspace/src

# Настройка рабочей директории
WORKDIR /workspace

# Инициализация рабочего пространства ROS
RUN colcon build

# Установка переменных окружения
ENV ROS_DOMAIN_ID=42

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Запуск SLAM при старте контейнера
CMD ["/entrypoint.sh"]
