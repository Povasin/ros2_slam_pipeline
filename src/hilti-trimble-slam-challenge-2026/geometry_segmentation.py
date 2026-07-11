# import os
# import sys
# import argparse
# import open3d as o3d
# import numpy as np

# os.environ['WAYLAND_DISPLAY'] = ''
# os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
# os.environ['GALLIUM_DRIVER'] = 'llvmpipe'

# def preprocess_point_cloud(pcd_path, voxel_size=0.05):
#     """Блок 1: Подготовка данных"""
#     print("\n" + "="*40)
#     print("БЛОК 1: ПРЕПРОЦЕССИНГ ОБЛАКА ТОЧЕК")
#     print("="*40)

#     print(f"[*] Чтение файла: {pcd_path}")
#     if not os.path.exists(pcd_path):
#         print("❌ ОШИБКА: Файл карты не найден!")
#         return None
        
#     pcd = o3d.io.read_point_cloud(pcd_path)
#     print(f"[*] Исходное количество точек: {len(pcd.points)}")

#     print(f"[*] Сжатие облака (размер вокселя: {voxel_size} м)...")
#     pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
#     print(f"[*] Точек после сжатия: {len(pcd_down.points)}")

#     print("[*] Удаление шума (Statistical Outlier Removal)...")
#     cl, good_indices = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    
#     pcd_clean = pcd_down.select_by_index(good_indices)
#     print(f"[*] Точек после очистки от шума: {len(pcd_clean.points)}")

#     pcd_clean.paint_uniform_color([0.7, 0.7, 0.7])
#     return pcd_clean


# def extract_horizontal_planes(pcd, distance_threshold=0.2):
#     """Блок 2: Поиск пола и потолка (Обновленные параметры)"""
#     print("\n" + "="*40)
#     print("БЛОК 2: ПОИСК ПОЛА И ПОТОЛКА (RANSAC)")
#     print("="*40)
    
#     rest_pcd = pcd
#     horizontal_pcd = o3d.geometry.PointCloud()
#     other_pcd = o3d.geometry.PointCloud()
    
#     for i in range(5):
#         if len(rest_pcd.points) < 500:
#             break
            
#         # Увеличили порог до 20 см (0.2)
#         plane_model, inliers = rest_pcd.segment_plane(distance_threshold=distance_threshold,
#                                                       ransac_n=3,
#                                                       num_iterations=1500) # Увеличили количество попыток
#         [a, b, c, d] = plane_model
        
#         plane_points = rest_pcd.select_by_index(inliers)
#         rest_pcd = rest_pcd.select_by_index(inliers, invert=True)
        
#         # СМЯГЧИЛИ ПРОВЕРКУ: Теперь горизонталью считается всё, где Z > 0.5 (допуск наклона почти 45 градусов!)
#         if abs(c) > 0.5:
#             print(f"[*] Найдена горизонталь! Точек: {len(plane_points.points)}. Нормаль Z={c:.2f}")
#             plane_points.paint_uniform_color([0.0, 0.4, 0.8])
#             horizontal_pcd += plane_points
#         else:
#             other_pcd += plane_points
            
#     other_pcd += rest_pcd
#     print(f"[*] Итого точек горизонталей: {len(horizontal_pcd.points)}")
    
#     return horizontal_pcd, other_pcd


# def extract_vertical_planes(pcd, distance_threshold=0.2, max_walls=10):
#     """Блок 3: Поиск стен (Обновленные параметры)"""
#     print("\n" + "="*40)
#     print("БЛОК 3: ПОИСК СТЕН (RANSAC)")
#     print("="*40)
    
#     rest_pcd = pcd
#     walls_pcd = o3d.geometry.PointCloud()
#     obstacles_pcd = o3d.geometry.PointCloud()
    
#     for i in range(max_walls):
#         if len(rest_pcd.points) < 100:
#             break
            
#         plane_model, inliers = rest_pcd.segment_plane(distance_threshold=distance_threshold,
#                                                       ransac_n=3,
#                                                       num_iterations=1500)
#         [a, b, c, d] = plane_model
        
#         plane_points = rest_pcd.select_by_index(inliers)
#         rest_pcd = rest_pcd.select_by_index(inliers, invert=True)
        
#         # СМЯГЧИЛИ ПРОВЕРКУ: У стены Z должен быть меньше 0.5
#         if abs(c) < 0.5:
#             print(f"[*] Найдена стена {i+1}! Точек: {len(plane_points.points)}. Нормаль Z={c:.2f}")
#             color_intensity = max(0.3, 0.9 - (i * 0.05))
#             plane_points.paint_uniform_color([0.0, color_intensity, 0.0])
#             walls_pcd += plane_points
#         else:
#             obstacles_pcd += plane_points
            
#     obstacles_pcd += rest_pcd
#     obstacles_pcd.paint_uniform_color([0.8, 0.1, 0.1])
    
#     print(f"[*] Итого точек стен: {len(walls_pcd.points)}")
#     print(f"[*] Итого точек препятствий: {len(obstacles_pcd.points)}")
    
#     return walls_pcd, obstacles_pcd

# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description="Семантическая сегментация 3D-карты")
#     parser.add_argument("floor", type=str, nargs="?", default="floor_1", help="Этаж (например, floor_1)")
#     parser.add_argument("date", type=str, nargs="?", default="2025-05-05", help="Дата (например, 2025-05-05)")
#     parser.add_argument("run", type=str, nargs="?", default="run_1", help="Прогон (например, run_1)")
#     args = parser.parse_args()

#     run_name = f"{args.floor}_{args.date}_{args.run}"

#     script_dir = os.path.dirname(os.path.abspath(__file__))
#     ws_root = os.path.abspath(os.path.join(script_dir, '../..'))
    
#     map_filename = f"map_{run_name}.pcd"
#     dynamic_map_path = os.path.join(ws_root, "data", map_filename)

#     # 1. Очистка
#     clean_cloud = preprocess_point_cloud(dynamic_map_path)
    
#     if clean_cloud:
#         # 2. Выделяем пол/потолок
#         floor_cloud, remaining_cloud = extract_horizontal_planes(clean_cloud)
        
#         # 3. Выделяем стены и препятствия из оставшихся точек
#         walls_cloud, obstacles_cloud = extract_vertical_planes(remaining_cloud)
        
#         print("\n[*] Открываем финальный результат: Синий - Пол, Зеленый - Стены, Красный - Объекты/Шум")
        
#         o3d.visualization.draw_geometries([floor_cloud, walls_cloud, obstacles_cloud], 
#         window_name=f"Семантическая 3D Карта ({run_name})")