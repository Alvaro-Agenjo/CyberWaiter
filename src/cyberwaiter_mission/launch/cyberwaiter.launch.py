import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    package_name = 'cyberwaiter_mission'
    
    # 1. Localizar interfaz.py dentro del directorio de instalación (lib)
    pkg_prefix = get_package_prefix(package_name)
    interfaz_path = os.path.join(pkg_prefix, 'lib', package_name, 'interfaz.py')

    # 2. Localizar directorio de RealSense para incluir su launch
    realsense_dir = get_package_share_directory('realsense2_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_depth': 'true'
        }.items()
    )

    # 3. Ruta absoluta al script de detección en la carpeta vision
    detector_script = '/home/lucia/Escritorio/CyberWaiter/vision/scripts/multi_object_detector.py'

    return LaunchDescription([
        # A. Lanzar Cámara RealSense
        camera_launch,

        # B. Lanzar Detector de Objetos por Visión (Python)
        ExecuteProcess(
            cmd=['python3', detector_script],
            output='screen'
        ),

        # C. Lanzar Interfaz de Usuario Solara (Web App)
        ExecuteProcess(
            cmd=['solara', 'run', interfaz_path, '--port', '8000'],
            output='screen'
        ),
        
        # D. Lanzar Nodo de Control de Misión (C++)
        Node(
            package=package_name,
            executable='Mision_Control',
            name='MisionController',
            output='screen'
        )
    ])
