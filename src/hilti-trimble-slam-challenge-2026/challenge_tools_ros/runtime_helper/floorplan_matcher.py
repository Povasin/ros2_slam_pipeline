#!/usr/bin/env python3
"""
ROS 2 Node for Floorplan-Anchored Localization using 2D ICP.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import open3d as o3d
from geometry_msgs.msg import PoseStamped

class FloorplanMatcher(Node):
    def __init__(self):
        super().__init__('floorplan_matcher')
        
        # Параметры чертежа
        self.declare_parameter('floorplan_path', '')
        self.floorplan_path = self.get_parameter('floorplan_path').value
        
        # Загрузка точек чертежа
        self.floorplan_points = self.load_floorplan()

        # Подписка на 2D-карту из map_builder
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/bev_map',
            self.map_callback,
            10
        )
        
        # Паблишер скорректированной позы (результат ICP)
        self.pose_publisher = self.create_publisher(PoseStamped, '/icp_corrected_pose', 10)

    def load_floorplan(self):
        """
        Загружает чертеж и конвертирует стены в 2D облако точек.
        """
        if not self.floorplan_path:
            self.get_logger().warn('Путь к чертежу не указан! ICP будет пропущен.')
            return None
            
        # Здесь будет логика загрузки PNG или DXF
        # В MVP-SLAM используется парсинг DXF для извлечения сегментов стен,
        # но для начала можно использовать бинаризованный PNG.
        self.get_logger().info(f'Загрузка чертежа из: {self.floorplan_path}')
        return o3d.geometry.PointCloud() # Заглушка: возвращаем пустой объект Open3D

    def map_callback(self, msg: OccupancyGrid):
        """
        Обрабатывает входящую BEV карту и запускает ICP с планом этажа.
        """
        if self.floorplan_points is None:
            return

        # 1. Извлечение координат стен (значение 100) из OccupancyGrid
        grid_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        y_indices, x_indices = np.where(grid_data == 100)
        
        if len(x_indices) < 50:
            # Слишком мало точек для надежного ICP
            return

        # 2. Перевод индексов сетки в реальные метры
        x_coords = (x_indices * msg.info.resolution) + msg.info.origin.position.x
        y_coords = (y_indices * msg.info.resolution) + msg.info.origin.position.y
        z_coords = np.zeros_like(x_coords)
        
        bev_points = np.vstack((x_coords, y_coords, z_coords)).T
        
        # 3. Конвертация в формат Open3D
        source_cloud = o3d.geometry.PointCloud()
        source_cloud.points = o3d.utility.Vector3dVector(bev_points)

        # 4. Выполнение алгоритма ICP
        self.run_icp(source_cloud)

    def run_icp(self, source_cloud):
        """
        Выполняет 2D ICP по осям (X, Y, Yaw).
        """
        # Начальное приближение (warm-start из SLAM)
        initial_guess = np.eye(4) 
        
        # Настройка параметров ICP (допуск и максимальное расстояние совпадения)
        threshold = 0.5 # Ищем совпадения в радиусе 0.5 метра
        
        # Запуск ICP
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source_cloud, self.floorplan_points, threshold, initial_guess,
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )
        
        if reg_p2p.fitness > 0.5: # Проверка качества совпадения
            self.get_logger().info(f'ICP успешен! Fitness: {reg_p2p.fitness:.2f}')
            # Трансформационная матрица reg_p2p.transformation содержит смещение.
            # Позже мы будем публиковать её в tf2.
        else:
            self.get_logger().debug('ICP не нашел надежных совпадений.')

def main(args=None):
    rclpy.init(args=args)
    node = FloorplanMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()