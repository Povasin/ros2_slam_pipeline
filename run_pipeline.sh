#!/bin/bash

FLOOR=${1:-floor_1}
DATE=${2:-2025-05-05}
RUN=${3:-run_1}
RUN_NAME="${FLOOR}_${DATE}_${RUN}"

echo "====================================="
echo "🚀 ЗАПУСК ПАЙПЛАЙНА ДЛЯ: $RUN_NAME 🚀"
echo "====================================="

BAG_DIR="/home/kirill_fdx/ros2_ws/data/${FLOOR}/${DATE}/${RUN}/rosbag"
if [ ! -d "$BAG_DIR" ]; then
    echo "🛑 ОШИБКА: Папка с rosbag не найдена!"
    echo "Путь: $BAG_DIR"
    exit 1
fi

export BAG_PATH="/home/kirill_fdx/ros2_ws/data/${FLOOR}/${DATE}/${RUN}/rosbag_pano"
export TRAJ_PATH="/home/kirill_fdx/ros2_ws/data/trajectory_${FLOOR}.txt"
export OUT_IMG_DIR="/home/kirill_fdx/ros2_ws/dataset/${FLOOR}/images"
export OUT_CSV="/home/kirill_fdx/ros2_ws/dataset/${FLOOR}/sync_index.csv"

# Удаляем старые файлы, чтобы скрипт ждал именно новую карту
rm -f /home/kirill_fdx/ros2_ws/data/map.pcd 2>/dev/null
rm -f /home/kirill_fdx/ros2_ws/data/trajectory.txt 2>/dev/null

# Активация ROS 2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rm -f /dev/shm/ros2_shm_* 2>/dev/null
export WAYLAND_DISPLAY=""

# ФИКС ПУТЕЙ: Жестко переходим в рабочую папку, чтобы алгоритмы знали, где они
cd /home/kirill_fdx/ros2_ws

echo "[1/5] Запуск сервера карты ($RUN_NAME)..."
ros2 launch challenge_tools_ros map_server.launch.py mask:=masks_with_windows run_name:=${RUN_NAME} < /dev/null &
MAP_PID=$!

sleep 2

echo "[2/5] Запуск OpenVINS, Map Builder и Logger..."
ros2 launch challenge_tools_ros run_openvins.launch.py < /dev/null &
SLAM_PID=$!

sleep 3

# ФИКС СИГНАЛОВ: Используем SIGTERM вместо SIGINT
trap 'echo -e "\n🛑 Экстренная остановка..."; kill -TERM $SLAM_PID $MAP_PID 2>/dev/null; exit' SIGINT SIGTERM

echo "[3/5] Проигрывание данных (rosbag)..."
ros2 bag play "$BAG_DIR" -r 0.7

echo "[4/5] Сохранение файлов (Отправляем SIGTERM)..."
# Шлем правильный системный сигнал, который игнорировать нельзя
kill -TERM $SLAM_PID
kill -TERM $MAP_PID

# Дублируем сигнал напрямую скрипту-строителю карты (на всякий случай)
pkill -TERM -f map_builder.py 2>/dev/null

echo "⏳ Идет расчет 3D-карты. Ждем появления map.pcd на диске..."

# Ждем карту, но добавляем счетчик (максимум 30 секунд ожидания), 
# чтобы скрипт не завис навсегда, если что-то пойдет не так
COUNTER=0
while [ ! -f /home/kirill_fdx/ros2_ws/data/map.pcd ] && [ $COUNTER -lt 30 ]; do
    sleep 1
    COUNTER=$((COUNTER+1))
done

if [ -f /home/kirill_fdx/ros2_ws/data/map.pcd ]; then
    echo "✅ Карта успешно сохранена! (Ждали $COUNTER сек.)"
else
    echo "❌ ОШИБКА: Карта map.pcd так и не появилась за 30 секунд!"
fi

sleep 2

# Очищаем процессы, если они всё еще висят
kill -9 $SLAM_PID $MAP_PID 2>/dev/null

if [ -f /home/kirill_fdx/ros2_ws/data/trajectory.txt ]; then
    mv /home/kirill_fdx/ros2_ws/data/trajectory.txt $TRAJ_PATH
fi

echo "[5/5] Синхронизация панорам (build_index.py)..."
cd ~/ros2_ws/src/hilti-trimble-slam-challenge-2026
python3 build_index.py

echo "✨ Запуск 3D-плеера..."
cd ~/ros2_ws/src/hilti-trimble-slam-challenge-2026
LIBGL_ALWAYS_SOFTWARE=1 python3 slam_viewer.py

echo "====================================="
echo "✅ ГОТОВО! ✅"
echo "====================================="