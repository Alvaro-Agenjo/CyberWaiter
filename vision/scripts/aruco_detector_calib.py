#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros

class ArucoDetectorCalib(Node):
    def __init__(self):
        super().__init__('aruco_detector_calib_node')
        
        # Parámetros del marcador (Ajusta la longitud del lado del cuadrado impreso en metros)
        self.marker_length = 0.175  # Lado del marcador impreso (en metros)
        self.aruco_dict_type = cv2.aruco.DICT_5X5_100
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # TF Broadcaster para enviar la posición al sistema de ROS
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Suscriptores
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)
        self.img_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        
        # Publicadores de diagnóstico visual
        self.image_pub = self.create_publisher(Image, '/aruco/image_output', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        
        # Configurar el diccionario y los parámetros de ArUco de forma moderna (OpenCV 4.7+)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Definir los puntos 3D del marcador en su propio sistema de coordenadas (origen en el centro)
        s = self.marker_length
        self.marker_3d_pts = np.array([
            [-s/2,  s/2, 0.0],  # Esquina superior izquierda
            [ s/2,  s/2, 0.0],  # Esquina superior derecha
            [ s/2, -s/2, 0.0],  # Esquina inferior derecha
            [-s/2, -s/2, 0.0]   # Esquina inferior izquierda
        ], dtype=np.float32)
        
        self.get_logger().info("Nodo detector de ArUco iniciado. Esperando calibración de cámara...")

    def info_callback(self, msg):
        # Lee la matriz K y distorsión D de la cámara de forma dinámica y una sola vez
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("--- ¡Parámetros de calibración de la cámara cargados exitosamente! ---")
            self.get_logger().info(f"Matriz K:\n{self.camera_matrix}")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return # Esperar a tener la info de la cámara
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detecta los marcadores usando el ArucoDetector moderno
            corners, ids, rejected = self.detector.detectMarkers(gray)
            
            if ids is not None and len(ids) > 0:
                for i in range(len(ids)):
                    marker_corners = corners[i][0] # Forma: (4, 2)
                    
                    # Dibujar borde verde alrededor del ArUco detectado
                    pts = marker_corners.astype(np.int32).reshape((-1, 1, 2))
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                    
                    # Dibujar el ID del marcador en la imagen
                    cx = int(np.mean(marker_corners[:, 0]))
                    cy = int(np.mean(marker_corners[:, 1]))
                    cv2.putText(frame, f"ID: {ids[i][0]}", (cx - 20, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # Estimar la pose 3D del marcador usando SolvePnP
                    success, rvec, tvec = cv2.solvePnP(
                        self.marker_3d_pts, marker_corners, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    
                    if success:
                        tx, ty, tz = tvec[0][0], tvec[1][0], tvec[2][0]
                        
                        # Proyectar ejes 3D sobre la imagen para visualización de diagnóstico
                        axis_len = self.marker_length * 0.5
                        axis_pts = np.array([
                            [0.0, 0.0, 0.0],
                            [axis_len, 0.0, 0.0],
                            [0.0, axis_len, 0.0],
                            [0.0, 0.0, axis_len]
                        ], dtype=np.float32)
                        
                        img_pts, _ = cv2.projectPoints(axis_pts, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                        origin = tuple(img_pts[0][0].astype(int))
                        x_end = tuple(img_pts[1][0].astype(int))
                        y_end = tuple(img_pts[2][0].astype(int))
                        z_end = tuple(img_pts[3][0].astype(int))
                        
                        cv2.line(frame, origin, x_end, (0, 0, 255), 3) # Eje X: Rojo
                        cv2.line(frame, origin, y_end, (0, 255, 0), 3) # Eje Y: Verde
                        cv2.line(frame, origin, z_end, (255, 0, 0), 3) # Eje Z: Azul
                        
                        # Publicar la Pose en topic
                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = "camera_color_optical_frame"
                        pose_msg.pose.position.x = tx
                        pose_msg.pose.position.y = ty
                        pose_msg.pose.position.z = tz
                        
                        # Convertir vector de rotación (rvec Rodrigues) a Cuaternión para ROS 2
                        rmat, _ = cv2.Rodrigues(rvec)
                        T = np.eye(4)
                        T[:3, :3] = rmat
                        
                        # Convertir matriz homogénea a cuaterniones
                        qw = np.sqrt(max(0.0, 1.0 + T[0,0] + T[1,1] + T[2,2])) / 2.0
                        qx = (T[2,1] - T[1,2]) / (4.0 * qw) if qw != 0 else 0.0
                        qy = (T[0,2] - T[2,0]) / (4.0 * qw) if qw != 0 else 0.0
                        qz = (T[1,0] - T[0,1]) / (4.0 * qw) if qw != 0 else 0.0
                        
                        pose_msg.pose.orientation.x = qx
                        pose_msg.pose.orientation.y = qy
                        pose_msg.pose.orientation.z = qz
                        pose_msg.pose.orientation.w = qw
                        self.pose_pub.publish(pose_msg)
                        
                        # Publicar la Transformada dinámica al árbol de TFs
                        t_msg = TransformStamped()
                        t_msg.header.stamp = self.get_clock().now().to_msg()
                        t_msg.header.frame_id = "camera_color_optical_frame"
                        t_msg.child_frame_id = "marker_frame"
                        t_msg.transform.translation.x = tx
                        t_msg.transform.translation.y = ty
                        t_msg.transform.translation.z = tz
                        t_msg.transform.rotation.x = qx
                        t_msg.transform.rotation.y = qy
                        t_msg.transform.rotation.z = qz
                        t_msg.transform.rotation.w = qw
                        self.tf_broadcaster.sendTransform(t_msg)
                    
            # Publica la imagen procesada de diagnóstico
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            
        except Exception as e:
            self.get_logger().error(f"Error en procesamiento de imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Eliminar llamada a rclpy.shutdown() duplicada para evitar RCLError en la salida
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
