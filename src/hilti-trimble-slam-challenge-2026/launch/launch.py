from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Получаем путь к директории, где лежит этот файл
    launch_dir = os.path.dirname(os.path.abspath(__file__))

    return LaunchDescription([
        # 1. Запуск OpenVINS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'run_openvins.launch.py'))
        ),
        
        # 2. Запуск Map Server с аргументами
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'map_server.launch.py')),
            launch_arguments={'mask': 'masks_with_windows', 'run_name': 'floor_1_2025-05-05_run_1'}.items()
        ),
        
        # 3. Запуск GroundTruth с аргументами
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'groundtruth_server.launch.py')),
            launch_arguments={'run_name': 'floor_1_2025-05-05_run_1'}.items()
        ),
    ])