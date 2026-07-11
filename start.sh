#!/bin/bash

cd "$(dirname "$0")"

# 1. Проверяем, существует ли Docker-образ
if [[ "$(sudo docker images -q slam_hilti_project:latest 2> /dev/null)" == "" ]]; then
    echo "📦 Docker-образ не найден. Начинаем сборку..."
    sudo docker build -t slam_hilti_project:latest .
fi

# Даем доступ к экрану
xhost + > /dev/null 2>&1

# 2. Определяем, что делать внутри контейнера
if [ "$#" -eq 0 ]; then
    echo "💻 Вход в интерактивный режим Docker..."
    DOCKER_CMD="bash"
else
    echo "🚀 Запуск автоматического пайплайна..."
    DOCKER_CMD="source /opt/ros/jazzy/setup.bash && \
                if [ ! -f 'install/setup.bash' ]; then \
                    echo '🛠️ Первая компиляция...'; \
                    colcon build --symlink-install; \
                fi && \
                source install/setup.bash && \
                ./run_pipeline.sh $@"
fi

# 3. Финальный запуск (Точная настройка для WSLg + Docker Desktop на Windows 11)
sudo docker run -it --rm \
    --name slam_container \
    --net=host \
    --ipc=host \
    --env="DISPLAY=$DISPLAY" \
    --env="WAYLAND_DISPLAY=$WAYLAND_DISPLAY" \
    --env="XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="LIBGL_ALWAYS_SOFTWARE=1" \
    --volume="/mnt/wslg/.X11-unix:/tmp/.X11-unix:ro" \
    --volume="/mnt/wslg:/mnt/wslg:ro" \
    --volume="$PWD:/ros2_ws" \
    slam_hilti_project:latest \
    bash -c "$DOCKER_CMD"