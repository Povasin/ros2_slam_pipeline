import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R_sci

def get_manual_transform_map(pts_slam: np.ndarray, pts_ref: np.ndarray):
    print("\n" + "="*50)
    print("ШАГ 1: ВЫРАВНИВАНИЕ КАРТЫ (4 КЛИКА)")
    print("="*50)
    print("1. На КРАСНОЙ карте: Начало стены -> Конец стены")
    print("2. На ЧЕРНОМ эталоне: Начало этой же стены -> Конец стены")
    
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.scatter(pts_ref[:, 0], pts_ref[:, 1], c='black', s=1, alpha=0.3, label='Эталон')
    ax.scatter(pts_slam[:, 0], pts_slam[:, 1], c='red', s=1, alpha=0.5, label='SLAM карта')
    ax.invert_yaxis()
    ax.axis('equal')
    ax.set_title("Шаг 1: Выравнивание карты по стене (4 клика)")
    ax.legend()

    clicks = plt.ginput(4, timeout=-1)
    plt.close(fig)

    if len(clicks) < 4:
        raise ValueError("Ошибка: Требуется ровно 4 клика!")

    p1_s, p2_s = np.array(clicks[0]), np.array(clicks[1])
    p1_r, p2_r = np.array(clicks[2]), np.array(clicks[3])

    vec_s = p2_s - p1_s
    vec_r = p2_r - p1_r

    scale = np.linalg.norm(vec_r) / np.linalg.norm(vec_s)
    theta = np.arctan2(vec_r[1], vec_r[0]) - np.arctan2(vec_s[1], vec_s[0])

    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    t = p1_r - scale * np.dot(R, p1_s)
    
    return scale, R, t

def get_manual_transform_trajectory(traj_raw: np.ndarray, pts_ref: np.ndarray, map_shape: tuple):
    print("\n" + "="*50)
    print("ШАГ 2: ВЫРАВНИВАНИЕ ТРАЕКТОРИИ (4 КЛИКА)")
    print("="*50)
    print("1. На ОРАНЖЕВОЙ траектории выберите длинный прямой участок пути.")
    print("   - Клик 1: Начало прямого пути")
    print("   - Клик 2: Конец прямого пути")
    print("2. На ЧЕРНОМ эталоне укажите этот же коридор В ТОМ ЖЕ ПОРЯДКЕ.")
    print("   - Клик 3: Начало коридора")
    print("   - Клик 4: Конец коридора")

    # Искусственно увеличиваем и смещаем метрическую траекторию, 
    # чтобы она была видна на фоне огромной пиксельной карты для кликов.
    vis_scale = 20.0
    vis_offset = np.array([map_shape[1] / 2, map_shape[0] / 2])
    traj_vis = traj_raw * vis_scale + vis_offset

    fig, ax = plt.subplots(figsize=(14, 10))
    ax.scatter(pts_ref[:, 0], pts_ref[:, 1], c='black', s=1, alpha=0.3, label='Эталон')
    ax.plot(traj_vis[:, 0], traj_vis[:, 1], c='darkorange', linewidth=2, label='Траектория (Сырая, для визуализации)')
    ax.scatter(traj_vis[0, 0], traj_vis[0, 1], c='lime', s=100, edgecolors='black', label='Точка старта', zorder=6)
    
    ax.invert_yaxis()
    ax.axis('equal')
    ax.set_title("Шаг 2: Выберите прямой путь на оранжевой линии, затем этот же коридор на черной карте")
    ax.legend()

    clicks = plt.ginput(4, timeout=-1)
    plt.close(fig)

    if len(clicks) < 4:
        raise ValueError("Ошибка: Требуется ровно 4 клика!")

    # Конвертируем пиксели кликов обратно в чистые метры для идеальной математики
    p1_raw = (np.array(clicks[0]) - vis_offset) / vis_scale
    p2_raw = (np.array(clicks[1]) - vis_offset) / vis_scale
    
    p3_ref = np.array(clicks[2])
    p4_ref = np.array(clicks[3])

    vec_raw = p2_raw - p1_raw
    vec_ref = p4_ref - p3_ref

    scale = np.linalg.norm(vec_ref) / np.linalg.norm(vec_raw)
    
    angle_raw = np.arctan2(vec_raw[1], vec_raw[0])
    angle_ref = np.arctan2(vec_ref[1], vec_ref[0])
    theta = angle_ref - angle_raw

    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    t = p3_ref - scale * np.dot(R, p1_raw)
    
    return scale, R, t, theta

def transform_trajectory(traj_path, out_path, scale, R_mat, t_vec, theta):
    data = np.loadtxt(traj_path, comments='#')
    timestamps = data[:, 0]
    xy_raw = data[:, 1:3]
    z_raw = data[:, 3]
    quaternions = data[:, 4:8]

    # Прямое применение вычисленной трансформации к метрам
    xy_aligned = scale * np.dot(xy_raw, R_mat.T) + t_vec
    z_aligned = z_raw * scale

    new_positions = np.column_stack((xy_aligned, z_aligned))

    rot_z = R_sci.from_euler('z', theta, degrees=False)
    new_rotations = rot_z * R_sci.from_quat(quaternions)

    aligned_data = np.column_stack((timestamps, new_positions, new_rotations.as_quat()))
    np.savetxt(out_path, aligned_data, fmt='%.6f', header="timestamp tx ty tz qx qy qz qw", comments='# ')
    return new_positions

def best_fit_transform(a, b):
    centroid_a, centroid_b = np.mean(a, axis=0), np.mean(b, axis=0)
    h = (a - centroid_a).T @ (b - centroid_b)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[1, :] *= -1
        r = vt.T @ u.T
    return r, centroid_b.T - np.dot(r, centroid_a.T)

def icp(src, dst, max_iterations=30, tolerance=1e-5):
    src_copy = np.copy(src)
    tree = KDTree(dst)
    prev_error = float('inf')
    for i in range(max_iterations):
        distances, indices = tree.query(src_copy)
        r, t = best_fit_transform(src_copy, dst[indices])
        src_copy = np.dot(src_copy, r.T) + t
        mean_error = np.mean(distances)
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
    return src_copy

def image_to_point_cloud(image_path, threshold=127):
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None: raise FileNotFoundError(f"Файл не найден: {image_path}")
    y, x = np.where(img < threshold)
    return np.column_stack((x, y)).astype(np.float64), img.shape

if __name__ == "__main__":
   # Укажи здесь путь к папке, где лежат твои файлы
    base_dir = "/home/kirill_fdx/ros2_ws/src/hilti-trimble-slam-challenge-2026/data/"
    
    # 1. Твой эталонный план этажа (из папки floorplans)
    path_ref = os.path.join(base_dir, "floorplans/masks_with_windows/floor_1.png")
    
    # 2. Твоя карта (которую ты сгенерировал в предыдущем шаге - картинкой)
    path_slam = os.path.join(base_dir, "my_first_floorplan.png") 
    
    # 3. Твоя исходная траектория
    path_traj_in = os.path.join(base_dir, "aligned_floor_1_slam_trajectory.txt")
    
    # 4. Имя файла, куда запишется ИДЕАЛЬНО выровненная траектория
    path_traj_out = os.path.join(base_dir, "SUPER_ALIGNED_traj.txt")

    pts_ref, ref_shape = image_to_point_cloud(path_ref)
    pts_slam_raw, _ = image_to_point_cloud(path_slam)

    # ШАГ 1: Выравниваем карту для красивого фона
    scale_map, R_map, t_map = get_manual_transform_map(pts_slam_raw, pts_ref)
    aligned_map_pts = icp(scale_map * np.dot(pts_slam_raw, R_map.T) + t_map, pts_ref)

    # Загружаем сырую траекторию
    data = np.loadtxt(path_traj_in, comments='#')
    traj_raw = data[:, 1:3]

    # ШАГ 2: Выравниваем саму траекторию независимо от карты
    scale_traj, R_traj, t_traj, theta_traj = get_manual_transform_trajectory(traj_raw, pts_ref, ref_shape)

    # Применение и сохранение
    aligned_traj_positions = transform_trajectory(
        path_traj_in, path_traj_out, scale_traj, R_traj, t_traj, theta_traj
    )

    # Финальная визуализация
    plt.figure(figsize=(14, 10))
    plt.scatter(pts_ref[:, 0], pts_ref[:, 1], c='black', s=1, alpha=0.3, label='Эталон')
    plt.scatter(aligned_map_pts[:, 0], aligned_map_pts[:, 1], c='blue', s=1, alpha=0.1, label='SLAM карта')
    
    plt.plot(aligned_traj_positions[:, 0], aligned_traj_positions[:, 1], c='darkorange', linewidth=2, label='Траектория (Идеальная)')
    plt.scatter(aligned_traj_positions[0, 0], aligned_traj_positions[0, 1], c='lime', s=100, edgecolors='black', label='Старт', zorder=6)
    
    plt.gca().invert_yaxis()
    plt.axis('equal') 
    plt.legend(loc='upper right')
    plt.title("Финальный результат: Траектория идеально вписана в коридоры")
    plt.show()