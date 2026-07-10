import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R_sci
from scipy.interpolate import interp1d

def robust_load_trajectory(filepath):
    """Надежная загрузка логов: игнорирует текст, понимает запятые и пробелы."""
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.replace(',', ' ').split()
            if len(parts) >= 4:
                try:
                    row = [float(x) for x in parts]
                    data.append(row)
                except ValueError:
                    continue
    return np.array(data)

def umeyama_alignment(X, Y):
    """Вычисление параметров масштаба, вращения и сдвига по алгоритму Умеямы."""
    n = X.shape[0]
    mu_x = np.mean(X, axis=0)
    mu_y = np.mean(Y, axis=0)
    
    X_centered = X - mu_x
    Y_centered = Y - mu_y
    
    Sigma = (Y_centered.T @ X_centered) / n
    U, D, Vt = np.linalg.svd(Sigma)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt.T) < 0:
        S[2, 2] = -1
        
    R = U @ S @ Vt
    
    # [ИСПРАВЛЕНО]: Корректный подсчет дисперсии (сумма квадратов по координатам x,y,z, усредненная по количеству точек)
    var_x = np.mean(np.sum(X_centered**2, axis=1))
    s = np.trace(np.diag(D) @ S) / var_x if var_x != 0 else 1.0
    
    t = mu_y - s * (R @ mu_x)
    return s, R, t

def align_trajectories(slam_path, gt_path, out_path):
    print(f"[*] Загрузка сырой траектории: {slam_path}")
    slam_data = robust_load_trajectory(slam_path)
    
    print(f"[*] Загрузка эталонной траектории: {gt_path}")
    gt_data = robust_load_trajectory(gt_path)

    # Вытаскиваем время (колонка 0) и координаты X, Y, Z (колонки 1, 2, 3)
    t_slam = slam_data[:, 0]
    X_slam_raw = slam_data[:, 1:4]
    
    t_gt = gt_data[:, 0]
    Y_gt_raw = gt_data[:, 1:4]

    # ИСПРАВЛЕНИЕ ЛОГИКИ ВРЕМЕНИ: Проверяем, не в наносекундах ли GT
    # Если GT время на несколько порядков больше (например > 10^17), переводим его в секунды
    if t_gt[0] > 1e15:
        print("[*] Обнаружено время в наносекундах в GT. Конвертируем в секунды...")
        t_gt = t_gt * 1e-9

    print("[*] Синхронизация траекторий по времени (Time-based matching)...")
    
    interp_func = interp1d(t_gt, Y_gt_raw, axis=0, bounds_error=False, fill_value=np.nan)
    Y_matched_full = interp_func(t_slam)

    # Оставляем только те точки, где SLAM и GT пересекаются по времени
    valid_mask = ~np.isnan(Y_matched_full).any(axis=1)
    X_slam_valid = X_slam_raw[valid_mask]
    Y_matched_valid = Y_matched_full[valid_mask]
    
    if np.sum(valid_mask) < 10:
        print("ОШИБКА: Слишком мало общих точек по времени! Проверьте синхронизацию.")
        return

    print(f"[*] Использовано {np.sum(valid_mask)} общих точек для расчета алгоритма Умеямы.")

    print("[*] Вычисление точного масштаба и сдвига (Умеяма)...")
    s, R, t = umeyama_alignment(X_slam_valid, Y_matched_valid)
    
    print(f"[*] Успешно вычислено: Масштаб = {s:.5f}")
    
    # Применяем вычисленные параметры
    X_aligned = s * (X_slam_raw @ R.T) + t

    print("[*] Пересчет кватернионов...")
    if slam_data.shape[1] >= 8:
        q_slam = slam_data[:, 4:8]
        r_transform = R_sci.from_matrix(R)
        q_aligned = [ (r_transform * R_sci.from_quat(q)).as_quat() for q in q_slam ]
        final_data = np.column_stack((slam_data[:, 0], X_aligned, q_aligned))
        header = "timestamp tx ty tz qx qy qz qw"
    else:
        final_data = np.column_stack((slam_data[:, 0], X_aligned))
        header = "timestamp tx ty tz"

    np.savetxt(out_path, final_data, fmt='%.6f', header=header, comments='# ')
    print(f"[SUCCESS] Файл сохранен:\n    {out_path}")
    
    # Визуализация
    plt.figure(figsize=(12, 10))
    plt.plot(Y_gt_raw[:, 0], Y_gt_raw[:, 1], c='black', linewidth=3, label='Эталон (GT)', alpha=0.5)
    plt.plot(X_aligned[:, 0], X_aligned[:, 1], c='darkorange', linewidth=2, label='Выровненный SLAM')
    
    plt.scatter(Y_gt_raw[0, 0], Y_gt_raw[0, 1], c='blue', s=100, label='Старт GT', zorder=5)
    plt.scatter(X_aligned[0, 0], X_aligned[0, 1], c='lime', s=100, label='Старт SLAM', zorder=5)
    
    plt.axis('equal')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend(loc='upper right')
    plt.title(f"Точное выравнивание по времени (Масштаб: x{s:.3f})")
    plt.show()

if __name__ == "__main__":
    slam_file = "/home/kirill_fdx/ros2_ws/data/trajectory.txt"
    gt_file = "/home/kirill_fdx/ros2_ws/src/hilti-trimble-slam-challenge-2026/groundtruth/floor_1_2025-05-05_run_1.txt"
    output_file = "/home/kirill_fdx/ros2_ws/data/aligned_floor_1_slam_trajectory.txt"
    
    if not os.path.exists(slam_file):
        print(f"ОШИБКА: Файл SLAM не найден по пути: {slam_file}")
    elif not os.path.exists(gt_file):
        print(f"ОШИБКА: Файл GT не найден по пути: {gt_file}")
    else:
        align_trajectories(slam_file, gt_file, output_file)