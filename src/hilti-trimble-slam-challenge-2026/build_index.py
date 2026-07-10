import os
import cv2
import numpy as np
import pandas as pd
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CompressedImage
import rosbag2_py

BAG_PATH = os.environ.get('BAG_PATH', './data/floor_1/2025-05-05/run_1/rosbag_pano')
TRAJ_PATH = os.environ.get('TRAJ_PATH', './data/trajectory_floor_1.txt')
OUT_IMG_DIR = os.environ.get('OUT_IMG_DIR', './dataset/floor_1/images')
OUT_CSV = os.environ.get('OUT_CSV', './dataset/floor_1/sync_index.csv')

def main():
    print("1. Создаем папки для датасета...")
    os.makedirs(OUT_IMG_DIR, exist_ok=True)

    print("2. Загружаем данные траектории (SLAM)...")
    traj_data = pd.read_csv(
        TRAJ_PATH, 
        sep=r'\s+', 
        comment='#', 
        header=None,
        names=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    )
    
    # Гарантируем, что время отсортировано (обязательно для быстрого бинарного поиска)
    traj_data = traj_data.sort_values(by='timestamp').reset_index(drop=True)
    timestamps = traj_data['timestamp'].values
    
    print(f"Загружено {len(traj_data)} точек траектории.")
    print(f"Время старта SLAM: {timestamps[0]:.3f}, финиша: {timestamps[-1]:.3f}")

    print("3. Открываем rosbag...")
    storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Ошибка при открытии rosbag: {e}")
        return

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    if IMAGE_TOPIC not in type_map:
        print(f"ОШИБКА: Топик '{IMAGE_TOPIC}' не найден в bag-файле!")
        return

    msg_type_str = type_map[IMAGE_TOPIC]
    print(f"Тип данных в топике: {msg_type_str}")

    storage_filter = rosbag2_py.StorageFilter(topics=[IMAGE_TOPIC])
    reader.set_filter(storage_filter)

    sync_records = []
    img_counter = 0
    lowest_diff = float('inf')
    first_frame_time = None
    frames_read = 0
    decode_errors = 0

    print("Начинаем извлечение и синхронизацию...")
    while reader.has_next():
        topic, data, t = reader.read_next()
        frames_read += 1
        
        if msg_type_str == 'sensor_msgs/msg/CompressedImage':
            msg = deserialize_message(data, CompressedImage)
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Логируем время самого первого кадра для диагностики
            if first_frame_time is None:
                first_frame_time = msg_time
                print(f"\n[ДИАГНОСТИКА] Время первого кадра в bag: {msg_time:.3f}")
                print(f"[ДИАГНОСТИКА] Смещение (offset) между стартом SLAM и камерой: {abs(timestamps[0] - msg_time):.3f} сек.\n")

            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            continue

        if img is None:
            decode_errors += 1
            continue

        idx = np.searchsorted(timestamps, msg_time)
        
        if idx == 0:
            min_idx = 0
        elif idx == len(timestamps):
            min_idx = len(timestamps) - 1
        else:
            diff_left = abs(timestamps[idx - 1] - msg_time)
            diff_right = abs(timestamps[idx] - msg_time)
            min_idx = idx - 1 if diff_left < diff_right else idx

        min_diff = abs(timestamps[min_idx] - msg_time)

        if min_diff < lowest_diff:
            lowest_diff = min_diff

        if min_diff < 0.1:
            img_name = f"frame_{img_counter:05d}.jpg"
            img_path = os.path.join(OUT_IMG_DIR, img_name)
            
            cv2.imwrite(img_path, img)

            row = traj_data.iloc[min_idx]
            sync_records.append({
                'id': img_counter,
                'image_path': img_path,
                'timestamp': msg_time,
                'x': row['x'],
                'y': row['y'],
                'z': row['z'],
                'qx': row['qx'],
                'qy': row['qy'],
                'qz': row['qz'],
                'qw': row['qw']
            })
            img_counter += 1

            if img_counter % 50 == 0:
                print(f"Синхронизировано {img_counter} кадров (Обработано сырых сообщений: {frames_read})...")

    print("\n================ ОТЧЕТ ================")
    print(f"Всего прочитано сообщений из топика: {frames_read}")
    print(f"Ошибок декодирования OpenCV: {decode_errors}")
    
    if lowest_diff != float('inf'):
        print(f"Самая маленькая разница во времени: {lowest_diff:.3f} сек.")
    else:
        print("Разница во времени не вычислялась (0 валидных кадров).")
    print("=======================================\n")

    print("4. Сохраняем CSV индекс...")
    if len(sync_records) > 0:
        df = pd.DataFrame(sync_records)
        df.to_csv(OUT_CSV, index=False)
        print(f"Готово! Сохранено {len(df)} кадров. Индекс лежит в {OUT_CSV}")
    else:
        print("ВНИМАНИЕ: Не удалось синхронизировать ни одного кадра.")
        
if __name__ == "__main__":
    main()