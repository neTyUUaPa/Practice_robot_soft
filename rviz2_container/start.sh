#!/bin/bash

# Запуск виртуального дисплея
Xvfb :1 -screen 0 1920x1080x24 &

# Установка переменной DISPLAY
export DISPLAY=:1

# Запуск RViz2
rviz2 &

# Запуск VNC-сервера
x11vnc -display :1 -forever -shared -passwd password &

# Запуск NoVNC
websockify --web=/usr/share/novnc 6080 localhost:5900