# Базовый образ с установленным ROS 2 Jazzy
FROM osrf/ros:jazzy-desktop

# Обновляем систему и ставим системные библиотеки (добавлены фиксы для RViz)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-pyqt5 \
    libgl1 \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxkbcommon-x11-0 \
    libqt5x11extras5 \
    libxcb-cursor0 \
    x11-apps \
    nano \
    libceres-dev \
    && rm -rf /var/lib/apt/lists/*

# Создаем виртуальное окружение
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv --system-site-packages $VIRTUAL_ENV

# Делаем виртуальное окружение активным по умолчанию
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Устанавливаем Python-библиотеки (добавлен фикс для NumPy)
RUN pip install \
    open3d \
    pandas \
    opencv-python-headless \
    evo \
    "numpy<2.0.0"

# Настраиваем воркспейс
ENV WS_DIR=/ros2_ws
WORKDIR /ros2_ws

# Добавляем автоматический source
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]