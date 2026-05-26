# Guía de Ejecución: Estimación de Pose 6D para Latas (ROS2)

Este sistema permite detectar la posición (XYZ) y orientación (RPY) de latas de refresco utilizando una cámara Intel RealSense y lógica geométrica robusta.

## 1. Requisitos Previos

### Hardware
* Cámara **Intel RealSense SR305** (o similar con soporte de profundidad).
* PC con **Ubuntu 22.04** y **ROS2 Humble** (recomendado).

### Software
Es necesario tener instalados los siguientes paquetes de ROS2:
```bash
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-cv-bridge
pip3 install opencv-python numpy
```

## 2. Puesta en Marcha

Sigue estos pasos en terminales separadas:

### Paso 1: Lanzar la cámara
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### Paso 2: Ejecutar el detector
```bash
python3 ~/Escritorio/LAyR/scripts/multi_object_detector.py
```

## 3. Protocolo de Operación (Bajo Demanda)

El sistema no publica datos hasta que se le asigna un objetivo.

### Cómo pedir una lata
Envía el nombre del objetivo al topic `/deteccion/objetivo`:
```bash
# Para Coca-Cola
ros2 topic pub /deteccion/objetivo std_msgs/msg/String "{data: 'COCA-COLA'}" --once

# Para Fanta Limón
ros2 topic pub /deteccion/objetivo std_msgs/msg/String "{data: 'FANTA-LIMON'}" --once

# Para Fanta Naranja
ros2 topic pub /deteccion/objetivo std_msgs/msg/String "{data: 'FANTA-NARANJA'}" --once

# Para Cerveza
ros2 topic pub /deteccion/objetivo std_msgs/msg/String "{data: 'CERVEZA'}" --once
```

### Cómo leer los resultados para el UR3
El sistema filtrará la estabilidad y publicará la respuesta final en dos topics de 3 valores cada uno:

* **Posición (Metros):** `ros2 topic echo /deteccion/posicion_final --once`
* **Orientación (Grados RPY):** `ros2 topic echo /deteccion/orientacion_final --once`

## 4. Interpretación de la Orientación (Ejes)
* **X (Roll):** Giro sobre el eje longitudinal de la lata.
* **Y (Pitch):** Inclinación (0° = De pie, 90° = Tumbada).
* **Z (Yaw):** Giro de la lata sobre la superficie de la mesa.

---
*Desarrollado para el sistema de pick-and-place con brazo robótico UR3.*
