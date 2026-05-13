import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    package_name = 'cyberwaiter_mission'
    
    # Localizar interfaz.py dentro del directorio de instalación (lib)
    pkg_prefix = get_package_prefix(package_name)
    interfaz_path = os.path.join(pkg_prefix, 'lib', package_name, 'interfaz.py')

    return LaunchDescription([
        # 1. Interfaz Solara
        ExecuteProcess(
            cmd=['solara', 'run', interfaz_path, '--port', '8000'],
            output='screen'
        ),
        
        # 2. Nodo de Misión (C++)
        Node(
            package=package_name,
            executable='Mision_Control',
            name='MisionController',
            output='screen'
        )
       
    ])
