#!/bin/bash

# Разрешаем локальные подключения к графическому серверу
xhost +local:root

cd "$(dirname "$0")"

# Настраиваем авторизацию для окон (RViz/Open3D)
if [ -z "$XAUTHORITY" ]; then
    XAUTH_PATH="$HOME/.Xauthority"
else
    XAUTH_PATH="$XAUTHORITY"
fi

echo "[*] Запуск Docker контейнера..."
sudo docker run -it --rm \
    --name slam_container \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=/root/.Xauthority" \
    --volume="$XAUTH_PATH:/root/.Xauthority:ro" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PWD:/ros2_ws" \
    slam_hilti_project:latest \
    bash