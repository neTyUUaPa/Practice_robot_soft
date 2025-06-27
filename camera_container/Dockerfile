# Базовый образ с Python и предустановленным OpenCV
FROM python:3.10-slim

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    python3 python3-pip \
    xvfb x11vnc novnc \
    libopencv-dev \
    #python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Установка Python-зависимостей через pip
COPY requirements.txt .
COPY Cv.py .
RUN pip install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple  -r requirements.txt

# Создание рабочей директории
WORKDIR /app

# Копирование вашего Python-скрипта в контейнер
COPY Cv.py .

# Команда для запуска скрипта
CMD ["python", "-u", "Cv.py"]
