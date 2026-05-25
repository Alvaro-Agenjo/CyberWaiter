# Plan de Acción: Calibración Extrínseca (Hand-Eye) UR3e y RealSense

Este plan detalla los pasos para realizar la calibración extrínseca (ojo en la mano - **Eye-in-Hand**) del robot real **UR3e** y la cámara **Intel RealSense**, basándonos en la configuración física y de red del repositorio `CyberWaiter`.

---

## 1. Conexión de Red y Configuración Física (UR3e Real)

Para controlar el robot y enviar/recibir poses, debemos configurar la interfaz de red Ethernet:

### Configuración en el Teach Pendant del UR3e:
1. Conecta el robot al ordenador mediante un cable Ethernet directo.
2. En el Teach Pendant, ve a **Settings > Network** y configura una IP estática en el mismo rango que el PC (ejemplo: `192.168.56.101`, máscara `255.255.255.0`).
3. Ve a **URCaps** y asegúrate de añadir y habilitar el programa de **External Control** configurando la IP del PC (host) y el puerto correspondiente.
4. Añade el nodo de control externo a tu programa del robot y dale a "Play".

### Configuración en el PC de Usuario:
Establece una dirección IPv4 estática en la tarjeta de red cableada en el mismo rango (ejemplo: `192.168.56.2`, máscara `255.255.255.0`).

---

## 2. Compilación del Controlador UR3e (`ros_humble_ur_driver`)

Usaremos el script recomendado por tu compañero (`partial_install.sh` de la rama `Mision` de CyberWaiter) para preparar el workspace del driver:

### Instalación Automática:
1. Creamos y ejecutamos el instalador parcial que descarga y compila el driver oficial de Universal Robots en `~/workspace/ros_humble_ur_driver`:
   ```bash
   mkdir -p ~/workspace && cd ~/workspace
   git clone -b Mision https://github.com/Alvaro-Agenjo/CyberWaiter.git
   cd CyberWaiter/Installs
   chmod +x partial_install.sh
   ./partial_install.sh
   ```
2. Una vez finalizado el build, haz un source al nuevo entorno:
   ```bash
   source ~/workspace/ros_humble_ur_driver/install/setup.bash
   ```

---

## 3. Extracción de la Cinemática Real y Lanzamiento

Cada robot UR3e tiene sutiles variaciones físicas de fábrica. Debemos extraer su archivo de calibración exacto para evitar errores de precisión al mover la pinza:

### Paso A: Extraer calibración cinemática del robot
```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.56.101 \
  target_filename:="${HOME}/my_ur3e_calibration.yaml"
```

### Paso B: Lanzar el driver del robot real
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101 \
  kinematics_params_file:="${HOME}/my_ur3e_calibration.yaml"
```

### Paso C: Activar el controlador de trayectorias
En una nueva terminal, activa el controlador para poder mover el brazo desde ROS2:
```bash
ros2 control set_controller_state scaled_joint_trajectory_controller active
```
*(Puedes verificar que esté activo usando `ros2 control list_controllers`)*.

---

## 4. Lanzamiento de Visión y Easy Handeye

Una vez que el robot está publicando sus TFs (`base_link` -> `tool0`) de forma nativa a través del driver, iniciamos la calibración extrínseca.

### Paso A: Lanzar Cámara RealSense
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### Paso B: Lanzar detector ArUco
```bash
source ~/ros2_ws/calibracion_ws/install/setup.bash
ros2 run aruco_ros single --ros-args \
  -p marker_id:=26 \
  -p marker_size:=0.1 \
  -p camera_frame:=camera_color_optical_frame \
  -p marker_frame:=aruco_marker_frame \
  -r /image:=/camera/color/image_raw \
  -r /camera_info:=/camera/color/camera_info
```

### Paso C: Lanzar la GUI de Calibración
```bash
source ~/ros2_ws/calibracion_ws/install/setup.bash
ros2 launch easy_handeye2 calibrate.launch.py \
  name:=mi_calibracion \
  calibration_type:=eye_in_hand \
  robot_base_frame:=base_link \
  robot_effector_frame:=tool0 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=aruco_marker_frame
```

---

## 5. Muestreo e Integración del Resultado

### Muestreo en la Interfaz Gráfica:
1. Abre `rqt_image_view` y selecciona `/aruco_single/result` para asegurarte de que ves los ejes sobre el marcador ArUco.
2. En la GUI de `easy_handeye`, mueve el UR3e a una pose y pulsa **"Take Sample"**.
3. Cambia la orientación y posición del UR3e (unas 12-15 poses diferentes, rotando la muñeca e inclinando la cámara, siempre manteniendo el ArUco visible). Toma muestra en cada pose.
4. Haz clic en **"Compute"** y luego en **"Save"**. El resultado se guardará en `~/.ros/easy_handeye/mi_calibracion.yaml`.

### Integración en `multi_object_detector.py`:
Modificaremos tu detector de latas para que en lugar de reportar poses en coordenadas de cámara, cargue esta calibración extrínseca mediante `tf2_ros` y publique las coordenadas directamente respecto a **`base_link`** (la base del robot UR3e).

---

## Preguntas y Confirmaciones para el Usuario

> [!IMPORTANT]
> 1. ¿Tu tarjeta de red Ethernet del PC ya está configurada con una IP estática compatible (ej. `192.168.56.2`)?
> 2. ¿El robot UR3e tiene la IP `192.168.56.101` asignada o debemos cambiarla en los comandos?
