import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Путь к файлу конфигурации фильтра
    package_name = 'velodyne'  # Пакет для Velodyne LiDAR
    # scan_filter_params = os.path.join(get_package_share_directory(package_name), 'config', 'scan_filter.yaml')  # Путь к конфигу

    return LaunchDescription([

        # Узел для работы с Velodyne LiDAR
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_driver',
            output='screen',
            parameters=[{
                'device_ip': '10.5.5.101',  # IP адрес лидара
                'frame_id': 'velodyne',  # Имя фрейма
                'port': 2368,  # Порт, на котором будет передаваться LIDAR данные
                'model': 'VLP16',  # Модель лидара
                'scan_mode': 'Standard',  # Можно указать другой режим, если нужно
                'rpm': 600  # Скорость вращения
            }]
        ),

    ])

