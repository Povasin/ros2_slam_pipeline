#!/bin/bash

# Автоматически определяем текущую директорию (должна быть корнем ros2_ws)
export WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

FLOOR=${1:-floor_1}
DATE=${2:-2025-05-05}
RUN=${3:-run_1}
RUN_NAME="${FLOOR}_${DATE}_${RUN}"

echo "====================================="
echo "🚀 ЗАПУСК ПАЙПЛАЙНА ДЛЯ: $RUN_NAME 🚀"
echo "Рабочая директория: $WS_DIR"
echo "====================================="

BAG_DIR="${WS_DIR}/data/${FLOOR}/${DATE}/${RUN}/rosbag"
if [ ! -d "$BAG_DIR" ]; then
    echo "🛑 ОШИБКА: Папка с rosbag не найдена!"
    echo "Путь: $BAG_DIR"
    exit 1
fi

# Экспортируем пути для Python-скриптов
export BAG_PATH="${WS_DIR}/data/${FLOOR}/${DATE}/${RUN}/rosbag_pano"
export TRAJ_PATH="${WS_DIR}/data/trajectory_${RUN_NAME}.txt"
export MAP_OUT_PATH="${WS_DIR}/data/map_${RUN_NAME}.pcd"
export OUT_IMG_DIR="${WS_DIR}/dataset/${FLOOR}/images"
export OUT_CSV="${WS_DIR}/dataset/${FLOOR}/sync_index.csv"

# Удаляем старые временные файлы
rm -f "${WS_DIR}/data/map.pcd" 2>/dev/null
rm -f "${WS_DIR}/data/trajectory.txt" 2>/dev/null

# Активация ROS 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rm -f /dev/shm/ros2_shm_* 2>/dev/null
export WAYLAND_DISPLAY=""

echo "[1/5] Запуск сервера карты ($RUN_NAME)..."
ros2 launch challenge_tools_ros map_server.launch.py mask:=masks_with_windows run_name:=${RUN_NAME} < /dev/null &
MAP_PID=$!
sleep 2

echo "[2/5] Запуск OpenVINS, Map Builder и Logger..."
ros2 launch challenge_tools_ros run_openvins.launch.py < /dev/null &
SLAM_PID=$!
sleep 3

trap 'echo -e "\n🛑 Экстренная остановка..."; kill -INT $SLAM_PID $MAP_PID 2>/dev/null; exit' SIGINT SIGTERM

echo "[3/5] Проигрывание данных (rosbag)..."
ros2 bag play "$BAG_DIR" -r 0.5

echo "[4/5] Сохранение файлов (Отправляем SIGINT для имитации Ctrl+C)..."
kill -INT $SLAM_PID
kill -INT $MAP_PID
pkill -INT -f map_builder.py 2>/dev/null

echo "⏳ Идет расчет 3D-карты. Ждем завершения..."

# Ждем пока нода сохранит карту по пути $MAP_OUT_PATH
COUNTER=0
while [ ! -f "$MAP_OUT_PATH" ] && [ $COUNTER -lt 30 ]; do
    sleep 1
    COUNTER=$((COUNTER+1))
done

if [ -f "$MAP_OUT_PATH" ]; then
    echo "✅ Карта успешно сохранена: map_${RUN_NAME}.pcd"
else
    echo "❌ ОШИБКА: Карта так и не появилась за 30 секунд!"
fi
sleep 2
kill -9 $SLAM_PID $MAP_PID 2>/dev/null

if [ -f "${WS_DIR}/data/trajectory.txt" ]; then
    mv "${WS_DIR}/data/trajectory.txt" "$TRAJ_PATH"
fi

echo "[5/5] Синхронизация панорам (build_index.py)..."
cd "${WS_DIR}/src/hilti-trimble-slam-challenge-2026"
python3 build_index.py

echo "✨ Запуск 3D-плеера..."
LIBGL_ALWAYS_SOFTWARE=1 WAYLAND_DISPLAY="" python3 slam_viewer.py

echo "====================================="
echo "✅ ГОТОВО! ✅"
echo "====================================="