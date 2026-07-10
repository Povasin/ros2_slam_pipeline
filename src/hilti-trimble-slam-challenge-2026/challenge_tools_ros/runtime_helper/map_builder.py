#!/usr/bin/env python3
"""
ROS 2 Node for building a global point cloud map from local SLAM outputs.
Adheres to PEP-8 standards.
"""
import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

class MapBuilder(Node):
    def __init__(self):
        super().__init__('map_builder')
        
        # Параметр размера вокселя (в метрах)
        self.declare_parameter('voxel_size', 0.2)
        self.voxel_size = self.get_parameter('voxel_size').value
        
        # Ограничения по высоте для выделения стен (в метрах)
        self.declare_parameter('z_min', 0.2)
        self.declare_parameter('z_max', 2.0)
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        
        # Объявление параметров системы координат
        self.declare_parameter('global_frame', 'global')
        self.declare_parameter('camera_frame', 'cam0')
        self.global_frame = self.get_parameter('global_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # Параметры 2D карты (OccupancyGrid)
        self.declare_parameter('resolution', 0.05)  # 5 см на ячейку
        self.declare_parameter('width', 2000)        # ширина в ячейках (10 метров)
        self.declare_parameter('height', 2000)       # высота в ячейках
        self.resolution = float(self.get_parameter('resolution').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        
        # Паблишер сетки стен
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/bev_map', qos)
        
        # Буфер трансформаций для получения матриц перехода между фреймами
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Подписка на локальное облако точек от OpenVINS
        self.subscription = self.create_subscription(
            PointCloud2,
            '/ov_msckf/points_slam',
            self.cloud_callback,
            10
        )

        # Массив для хранения агрегированных глобальных точек
        self.global_points = np.empty((0, 3), dtype=np.float32)

    def extract_bev(self, points: np.ndarray) -> np.ndarray:
        """
        Вырезает точки стен по оси Z и проецирует их в 2D (Bird's-Eye View).
        
        :param points: Массив глобальных точек размерностью (N, 3).
        :return: Массив 2D-точек размерностью (M, 2).
        """
        if points.shape[0] == 0:
            return np.empty((0, 2), dtype=np.float32)
            
        # Логическая маска: оставляем точки, высота которых попадает в диапазон
        height_mask = (points[:, 2] > self.z_min) & (points[:, 2] < self.z_max)
        wall_points = points[height_mask]
        
        if wall_points.shape[0] == 0:
            return np.empty((0, 2), dtype=np.float32)
            
        # Проецируем точки в 2D, отбрасывая координату Z
        bev_points = wall_points[:, :2]
        return bev_points

    def apply_voxel_grid_filter(self, points: np.ndarray) -> np.ndarray:
        """
        Осуществляет даунсэмплинг облака точек с использованием воксельной сетки.
        
        :param points: Массив точек размерностью (N, 3) типа float32.
        :return: Отфильтрованный массив точек.
        """
        if points.shape[0] == 0:
            return points
            
        # Квантование координат: делим на размер вокселя и округляем
        quantized_coords = np.round(points / self.voxel_size).astype(int)
        
        # Находим индексы уникальных вокселей
        _, unique_indices = np.unique(quantized_coords, axis=0, return_index=True)
        return points[unique_indices]

    def cloud_callback(self, msg: PointCloud2):
        """
        Обработка входящего облака точек: трансформация и добавление в карту.
        """
        self.get_logger().info('Получено облако точек! Строю карту...')
        try:
            # Получение матрицы трансформации на момент времени съемки кадра
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.1)
            )
            
            # Применение трансформации к облаку точек
            transformed_cloud = do_transform_cloud(msg, transform)
            
            # 1. Читаем точки как итератор
            points_iter = pc2.read_points(
                transformed_cloud, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            )
            
            unpacked_points = [[p[0], p[1], p[2]] for p in points_iter]
            
            if not unpacked_points:
                return

            # 3. Теперь NumPy без проблем сделает из этого 2D-матрицу чисел
            new_points_np = np.array(unpacked_points, dtype=np.float32)
            
            # 1. Фильтруем новые точки для скорости
            filtered_new_points = self.apply_voxel_grid_filter(new_points_np)
            # 2. Добавляем их к глобальному облаку
            if self.global_points.shape[0] == 0:
                self.global_points = filtered_new_points
            else:
                combined_points = np.vstack((self.global_points, filtered_new_points))
                # 3. Фильтруем объединенное облако
                self.global_points = self.apply_voxel_grid_filter(combined_points)
                
            # 4. Формируем 2D проекцию (BEV)
            bev_map = self.extract_bev(self.global_points)
            self.get_logger().info(f'BEV map contains {len(bev_map)} 2D points.')
            
            # Публикация сетки OccupancyGrid
            self.publish_global_map(msg.header.stamp)

        except Exception as e:
            self.get_logger().warn(f'Transformation failed: {e}')

    def publish_global_map(self, stamp):
        """
        Преобразует накопленные BEV точки в OccupancyGrid и публикует.
        """
        bev_points = self.extract_bev(self.global_points)
        if bev_points.shape[0] == 0:
            return

        # Инициализируем пустую сетку (значение 0 - свободно)
        grid_data = np.zeros(self.width * self.height, dtype=np.int8)

        # Проецируем точки в индексы сетки
        for x, y in bev_points:
            # Центрируем карту (origin в центре сетки)
            ix = int((x / self.resolution) + (self.width / 2))
            iy = int((y / self.resolution) + (self.height / 2))
            
            if 0 <= ix < self.width and 0 <= iy < self.height:
                grid_data[iy * self.width + ix] = 100  # 100 - занято (стена)

        # Создаем сообщение
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        
        msg.info.resolution = float(self.resolution)
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Сдвигаем карту, чтобы центр был в (0,0)
        msg.info.origin.position.x = float(- (self.width * self.resolution) / 2)
        msg.info.origin.position.y = float(- (self.height * self.resolution) / 2)
        
        msg.data = grid_data.tolist()
        self.map_publisher.publish(msg)
    def save_pcd_map(self, file_path='/home/kirill_fdx/ros2_ws/data/map.pcd'):
        """Сохраняет накопленное облако точек в PCD файл перед выходом."""
        if self.global_points.shape[0] > 0:
            self.get_logger().info(f'Сохранение карты из {self.global_points.shape[0]} точек...')
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.global_points)
            o3d.io.write_point_cloud(file_path, pcd)
            self.get_logger().info(f'УСПЕХ: Карта сохранена в {file_path}')
        else:
            self.get_logger().warn('Карта пуста, нечего сохранять.')


def main(args=None):
    rclpy.init(args=args)
    node = MapBuilder()
    try:
        rclpy.spin(node)
    except BaseException:
        # Перехватываем KeyboardInterrupt, ExternalShutdownException и другие системные сигналы
        pass
    finally:
        # Гарантированное сохранение при любом варианте остановки
        node.get_logger().info('Остановка ноды. Сохраняем данные...')
        node.save_pcd_map() 
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass # Игнорируем ошибку "rcl_shutdown already called", если ROS 2 уже закрыл контекст

if __name__ == '__main__':
    main()