import sys
import os
import glob
import cv2
import numpy as np
import pandas as pd
import open3d as o3d
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QSlider, QLabel, QPushButton)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

# ================= НАСТРОЙКИ ПУТЕЙ =================
WS_DIR = os.environ.get('WS_DIR', '.') 
CSV_PATH = os.environ.get('OUT_CSV', os.path.join(WS_DIR, 'dataset/floor_1/sync_index.csv'))
PCD_MAP_PATH = os.environ.get('MAP_OUT_PATH', os.path.join(WS_DIR, 'data/map.pcd'))
# ===================================================

class SLAMViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 1. Загрузка индекса
        print("Загрузка данных...")
        if not os.path.exists(CSV_PATH):
            print(f"ОШИБКА: Файл {CSV_PATH} не найден!")
            sys.exit(1)
            
        self.df = pd.read_csv(CSV_PATH)
        self.total_frames = len(self.df)
        print(f"Загружено {self.total_frames} кадров.")

        # 2. Настройка Open3D
        print("Инициализация 3D среды...")
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="3D Map (Open3D)", width=800, height=600)
        
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0.1, 0.1, 0.1])
        opt.point_size = 2.0
        
        # Пытаемся загрузить PCD карту
        if os.path.exists(PCD_MAP_PATH):
            self.pcd = o3d.io.read_point_cloud(PCD_MAP_PATH)
            self.vis.add_geometry(self.pcd)
            print("Карта успешно загружена.")
        else:
            print(f"ВНИМАНИЕ: Карта {PCD_MAP_PATH} не найдена. 3D сцена будет пустой.")
            self.pcd = None

        # ==========================================
        # НОВЫЙ БЛОК: ОТРИСОВКА ТРАЕКТОРИИ (ЛИНИИ)
        # ==========================================
        # Извлекаем все координаты из таблицы в numpy массив
        points = self.df[['x', 'y', 'z']].values
        
        if len(points) > 1:
            # Создаем линии, соединяющие точки (0-1, 1-2, 2-3 и т.д.)
            lines = [[i, i+1] for i in range(len(points)-1)]
            # Задаем яркий оранжевый цвет
            colors = [[1.0, 0.5, 0.0] for _ in range(len(lines))]
            
            self.trajectory_line = o3d.geometry.LineSet()
            self.trajectory_line.points = o3d.utility.Vector3dVector(points)
            self.trajectory_line.lines = o3d.utility.Vector2iVector(lines)
            self.trajectory_line.colors = o3d.utility.Vector3dVector(colors)
            
            self.vis.add_geometry(self.trajectory_line)
            print("Траектория успешно отрисована.")
        # ==========================================

        # Маркер текущей позиции (красная сфера)
        self.marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.3)
        self.marker.paint_uniform_color([1.0, 0.0, 0.0])
        
        first_row = self.df.iloc[0]
        self.marker.translate([first_row['x'], first_row['y'], first_row['z']])
        self.vis.add_geometry(self.marker)

        # 3. Настройка интерфейса PyQt
        self.setWindowTitle("SLAM Panoramic Synchronizer")
        self.setGeometry(100, 100, 1000, 600)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        self.info_label = QLabel("Frame: 0 | Pos: (0, 0, 0)")
        self.info_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(self.info_label)

        self.image_label = QLabel("Загрузка изображения...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumHeight(400)
        layout.addWidget(self.image_label)

        slider_layout = QHBoxLayout()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.total_frames - 1)
        self.slider.valueChanged.connect(self.on_slider_change)
        
        slider_layout.addWidget(QLabel("Start"))
        slider_layout.addWidget(self.slider)
        slider_layout.addWidget(QLabel("End"))
        layout.addLayout(slider_layout)

        # 4. Таймер для связки PyQt и Open3D
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_open3d)
        self.timer.start(30)

        self.on_slider_change(0)
        self.reset_camera_view()

    def on_slider_change(self, index):
        row = self.df.iloc[index]
        
        x, y, z = row['x'], row['y'], row['z']
        self.info_label.setText(f"Frame: {index}/{self.total_frames-1} | Pos: ({x:.2f}, {y:.2f}, {z:.2f})")

        img_path = row['image_path']
        if os.path.exists(img_path):
            cv_img = cv2.imread(img_path)
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = cv_img.shape
            bytes_per_line = ch * w
            qt_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
            pixmap = QPixmap.fromImage(qt_img).scaled(950, 450, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.image_label.setPixmap(pixmap)
        else:
            self.image_label.setText(f"Image not found: {img_path}")

        current_pos = self.marker.get_center()
        new_pos = np.array([x, y, z])
        translation = new_pos - current_pos
        self.marker.translate(translation)
        
        self.vis.update_geometry(self.marker)

    def update_open3d(self):
        self.vis.poll_events()
        self.vis.update_renderer()

    def reset_camera_view(self):
        view_control = self.vis.get_view_control()
        if self.pcd:
            view_control.set_front([0, 0, -1])
            view_control.set_up([0, -1, 0])
        else:
            view_control.set_lookat(self.marker.get_center())

    def closeEvent(self, event):
        self.vis.destroy_window()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = SLAMViewer()
    viewer.show()
    sys.exit(app.exec_())