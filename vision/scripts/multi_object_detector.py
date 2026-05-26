import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import math
from scipy.spatial.transform import Rotation as R

class MultiObjectDetector(Node):
    def __init__(self):
        super().__init__('multi_object_detector_node')
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/deteccion/output', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/deteccion/pose', 10)
        
        # Rutas relativas
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_path = os.path.abspath(os.path.join(script_dir, '..'))
        
        self.min_matches = 80
        self.latest_depth_frame = None
        self.current_objective = None
        
        self.targets = {
            'COCA-COLA': {
                'img_path': os.path.join(base_path, 'cola-normal.jpg'),
                'lower_color': np.array([150, 100, 50]), 
                'upper_color': np.array([179, 255, 255]),
                'box_color': (0, 255, 0)
            },
            'FANTA': {
                'img_path': os.path.join(base_path, 'fanta-limon.jpg'),
                'lower_color': np.array([20, 100, 100]),
                'upper_color': np.array([35, 255, 255]),
                'box_color': (0, 0, 255)
            }
        }

        # Historial para estabilización (Filtro de Mediana)
        self.history_size = 20
        self.history = {name: [] for name in self.targets.keys()}
        self.persistence = {name: 0 for name in self.targets.keys()}
        self.last_valid_pose = {name: None for name in self.targets.keys()}

        self.orb = cv2.ORB_create(nfeatures=1500)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # PROCESAMIENTO DE TARGETS
        for name, target in self.targets.items():
            img_ref = cv2.imread(target['img_path'], 0)
            if img_ref is not None:
                kp, des = self.orb.detectAndCompute(img_ref, None)
                target['des_ref'] = des
                target['kp_ref'] = kp
                target['h_ref'], target['w_ref'] = img_ref.shape[:2]
                self.get_logger().info(f'Target "{name}" cargado correctamente.')
            else:
                self.get_logger().error(f'No se pudo encontrar la imagen en: {target["img_path"]}')

        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.depth_subscription = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        # Suscriptor para el objetivo
        self.target_sub = self.create_subscription(String, '/deteccion/objetivo', self.target_callback, 10)
        # Publishers para la respuesta final (3 valores para posición y 3 para orientación)
        self.pos_final_pub = self.create_publisher(Vector3, '/deteccion/posicion_final', 10)
        self.ori_final_pub = self.create_publisher(Vector3, '/deteccion/orientacion_final', 10)
        
        # Suscriptor para la pose real del robot UR3
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.robot_pose_callback, 10)
        self.latest_robot_pose = None

        # Publicador de la pose final transformada en la base del robot
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/deteccion/posicion_robot', 10)

        self.get_logger().info("Sistema Bajo Demanda Iniciado. Esperando en /deteccion/objetivo...")

    def robot_pose_callback(self, msg):
        self.latest_robot_pose = msg

    def target_callback(self, msg):
        target_name = msg.data.upper()
        if target_name in self.targets:
            self.current_objective = target_name
            self.get_logger().info(f"Objetivo fijado: {target_name}. Buscando...")
        else:
            self.current_objective = None
            self.get_logger().info("Objetivo limpiado o no reconocido. Mostrando cámara en bruto.")

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        if self.current_objective is None:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = np.ones((15,15), np.uint8)
        
        name = self.current_objective
        target = self.targets[name]

        if 'des_ref' in target:
            mask_body = cv2.inRange(hsv, target['lower_color'], target['upper_color'])
            mask_body = cv2.morphologyEx(mask_body, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask_body, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 3000:
                    x, y, w, h = cv2.boundingRect(c)
                    
                    # 1. POSICIÓN
                    centro_u, centro_v = x + w/2, y + h/2
                    distancia_z = 0.5
                    if self.latest_depth_frame is not None:
                        v_idx, u_idx = int(centro_v), int(centro_u)
                        if 0 <= v_idx < self.latest_depth_frame.shape[0] and 0 <= u_idx < self.latest_depth_frame.shape[1]:
                            distancia_z = self.latest_depth_frame[v_idx, u_idx] / 1000.0

                    # Parámetros calibrados (1920x1080): fx=1337.45, fy=1327.61, cx=988.65, cy=550.17
                    #pos_x = (centro_u - 988.65) * distancia_z / 1337.45
                    #pos_y = (centro_v - 550.17) * distancia_z / 1327.61
                    #[fx, 0, cx, 0, fy, cy, 0, 0, 1]

                    if distancia_z <= 0: distancia_z = 0.4
                    pos_x = (centro_u - 965.0) * distancia_z / 1402.5
                    pos_y = (centro_v - 537.0) * distancia_z / 1402.5

                    # 2. ORIENTACIÓN (Ejes Estructurales)
                    rect = cv2.minAreaRect(c)
                    (rect_x, rect_y), (rect_w, rect_h), rect_angle = rect
                    
                    if rect_w > rect_h:
                        yaw = math.radians(rect_angle)
                        pitch = math.pi/2 # Tumbada
                    else:
                        yaw = math.radians(rect_angle + 90)
                        pitch = 0.0 # Vertical

                    # Cuaternión
                    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
                    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
                    qx, qy, qz, qw = 0.0, cp * sy, sp * cy, cp * cy
                    
                    q_norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
                    qx/=q_norm; qy/=q_norm; qz/=q_norm; qw/=q_norm

                    # 3. FILTRADO TEMPORAL
                    current_data = [pos_x, pos_y, distancia_z, qx, qy, qz, qw, rect_x, rect_y, rect_w, rect_h, rect_angle]
                    self.history[name].append(current_data)
                    if len(self.history[name]) > self.history_size: self.history[name].pop(0)

                    self.last_valid_pose[name] = np.median(self.history[name], axis=0)
                    self.persistence[name] = 10

        # Visualización y Publicación
        if self.current_objective and self.persistence[self.current_objective] > 0:
            name = self.current_objective
            target = self.targets[name]
            
            if self.persistence[name] > 0:
                self.persistence[name] -= 1
                m = self.last_valid_pose[name]
                f_x, f_y, f_z, f_qx, f_qy, f_qz, f_qw = m[0], m[1], m[2], m[3], m[4], m[5], m[6]
                f_rx, f_ry, f_rw, f_rh, f_ra = m[7], m[8], m[9], m[10], m[11]

                # Publicar Pose Provisoria
                p_msg = PoseStamped()
                p_msg.header.stamp = self.get_clock().now().to_msg()
                p_msg.header.frame_id = "camera_color_optical_frame"
                p_msg.pose.position.x, p_msg.pose.position.y, p_msg.pose.position.z = f_x, f_y, f_z
                p_msg.pose.orientation.x, p_msg.pose.orientation.y = f_qx, f_qy
                p_msg.pose.orientation.z, p_msg.pose.orientation.w = f_qz, f_qw
                self.pose_publisher.publish(p_msg)

                # 1. CÁLCULO DE EULER (RPY) PARA VISUALIZACIÓN Y ENVÍO
                sinr_cosp = 2 * (f_qw * f_qx + f_qy * f_qz)
                cosr_cosp = 1 - 2 * (f_qx * f_qx + f_qy * f_qy)
                f_roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                sinp = 2 * (f_qw * f_qy - f_qz * f_qx)
                f_pitch = math.degrees(math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi/2, sinp))

                siny_cosp = 2 * (f_qw * f_qz + f_qx * f_qy)
                cosy_cosp = 1 - 2 * (f_qy * f_qy + f_qz * f_qz)
                f_yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                # CHEQUEO DE ESTABILIDAD PARA ENVÍO FINAL
                h_array = np.array(self.history[name])
                is_stable = False
                if len(h_array) >= self.history_size:
                    variance = np.var(h_array[:, 3:7], axis=0).mean() 
                    if variance < 0.005: 
                        # Enviar Posición (X, Y, Z)
                        pos_msg = Vector3()
                        pos_msg.x, pos_msg.y, pos_msg.z = f_x, f_y, f_z
                        self.pos_final_pub.publish(pos_msg)
                        
                        # Enviar Orientación (Roll, Pitch, Yaw)
                        ori_msg = Vector3()
                        ori_msg.x, ori_msg.y, ori_msg.z = f_roll, f_pitch, f_yaw
                        self.ori_final_pub.publish(ori_msg)
                        
                        # ==========================================
                        # TRANSFORMACIÓN DE COORDENADAS EYE-IN-HAND
                        # ==========================================
                        try:
                            # 1. Posición y Rotación en el frame de la cámara
                            p_cam = np.array([f_x, f_y, f_z])
                            r_cam = R.from_quat([f_qx, f_qy, f_qz, f_qw])
                            
                            # 2. Transformar al frame del gripper (tool0) usando nuestra calibración de Park
                            t_c2g = np.array([0.09364, 0.00474, -0.13818])
                            q_c2g = [-0.00842, -0.00668, -0.55121, 0.83430]
                            r_c2g = R.from_quat(q_c2g)
                            
                            p_tool = r_c2g.apply(p_cam) + t_c2g
                            r_tool = r_c2g * r_cam
                            
                            # 3. Si tenemos la pose real del robot, transformar al frame de la base (base_link)
                            if self.latest_robot_pose is not None:
                                t_g2b = np.array([
                                    self.latest_robot_pose.pose.position.x,
                                    self.latest_robot_pose.pose.position.y,
                                    self.latest_robot_pose.pose.position.z
                                ])
                                q_g2b = [
                                    self.latest_robot_pose.pose.orientation.x,
                                    self.latest_robot_pose.pose.orientation.y,
                                    self.latest_robot_pose.pose.orientation.z,
                                    self.latest_robot_pose.pose.orientation.w
                                ]
                                r_g2b = R.from_quat(q_g2b)
                                
                                p_base = r_g2b.apply(p_tool) + t_g2b
                                r_base = r_g2b * r_tool
                                q_base = r_base.as_quat()
                                
                                # Publicar en topic de test
                                robot_pose_msg = PoseStamped()
                                robot_pose_msg.header.stamp = self.get_clock().now().to_msg()
                                robot_pose_msg.header.frame_id = "base_link"
                                robot_pose_msg.pose.position.x = p_base[0]
                                robot_pose_msg.pose.position.y = p_base[1]
                                robot_pose_msg.pose.position.z = p_base[2]
                                robot_pose_msg.pose.orientation.x = q_base[0]
                                robot_pose_msg.pose.orientation.y = q_base[1]
                                robot_pose_msg.pose.orientation.z = q_base[2]
                                robot_pose_msg.pose.orientation.w = q_base[3]
                                self.robot_pose_pub.publish(robot_pose_msg)
                                
                                # Limitar log a una vez por segundo
                                now_sec = self.get_clock().now().nanoseconds / 1e9
                                if not hasattr(self, 'last_verify_log_time') or (now_sec - self.last_verify_log_time) >= 1.0:
                                    self.last_verify_log_time = now_sec
                                    self.get_logger().info(
                                        f"🎯 [VERIFICACIÓN] {name} detectada!\n"
                                        f"   -> EN CÁMARA (camera_color_optical_frame): X={f_x:.4f}, Y={f_y:.4f}, Z={f_z:.4f}\n"
                                        f"   -> EN ROBOT BASE (base_link):               X={p_base[0]:.4f}, Y={p_base[1]:.4f}, Z={p_base[2]:.4f}"
                                    )
                            else:
                                # Log si no hay pose de robot
                                now_sec = self.get_clock().now().nanoseconds / 1e9
                                if not hasattr(self, 'last_verify_log_time') or (now_sec - self.last_verify_log_time) >= 1.0:
                                    self.last_verify_log_time = now_sec
                                    self.get_logger().warn("⚠️ Esperando pose del robot en /tcp_pose_broadcaster/pose...")
                        except Exception as e:
                            self.get_logger().error(f"Error al transformar pose: {e}")
                        
                        is_stable = True

                # 2. EJES 3D
                center_img = (int(f_rx), int(f_ry))
                ang_z = math.radians(f_ra if f_rw > f_rh else f_ra + 90)
                end_z = (int(f_rx + 60 * math.cos(ang_z)), int(f_ry + 60 * math.sin(ang_z)))
                end_x = (int(f_rx + 60 * math.cos(ang_z + math.pi/2)), int(f_ry + 60 * math.sin(ang_z + math.pi/2)))
                y_tilt = math.sin(math.radians(f_pitch))
                end_y = (int(f_rx - 40 * y_tilt * math.sin(ang_z)), int(f_ry + 40 * y_tilt * math.cos(ang_z)))

                cv2.line(frame, center_img, end_z, (255, 0, 0), 4) # Z - Azul
                cv2.line(frame, center_img, end_x, (0, 0, 255), 4) # X - Rojo
                cv2.line(frame, center_img, end_y, (0, 255, 0), 4) # Y - Verde
                cv2.circle(frame, center_img, 5, (255, 255, 255), -1)

                rect_points = np.int0(cv2.boxPoints(((f_rx, f_ry), (f_rw, f_rh), f_ra)))
                cv2.drawContours(frame, [rect_points], 0, self.targets[name]['box_color'], 2)
                
                # Etiquetas
                status_text = "ESTABLE - ENVIADO" if is_stable else "BUSCANDO..."
                cv2.putText(frame, f"{name} - {status_text}", (int(f_rx-50), int(f_ry-f_rh/2-60)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.targets[name]['box_color'], 2)
                cv2.putText(frame, f"XYZ: [{f_x:.2f}, {f_y:.2f}, {f_z:.2f}]m", 
                            (int(f_rx-50), int(f_ry-f_rh/2-40)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
                cv2.putText(frame, f"RPY: [{f_roll:.0f}, {f_pitch:.0f}, {f_yaw:.0f}] deg", 
                            (int(f_rx-50), int(f_ry-f_rh/2-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,255,200), 2)

        self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def depth_callback(self, data):
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

def main():
    rclpy.init()
    node = MultiObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
