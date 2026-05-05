import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class HybridVisionPro(Node):
    def __init__(self):
        super().__init__('hybrid_vision_pro_node')
        self.bridge = CvBridge()
        
        # 1. Carga de referencia y extractor de puntos (ORB)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_path = os.path.abspath(os.path.join(script_dir, '..'))
        ruta_imagen = os.path.join(base_path, 'reference.jpg')

        self.template = cv2.imread(ruta_imagen, 0)
        
        self.orb = cv2.ORB_create(nfeatures=500)
        self.kp_ref, self.des_ref = self.orb.detectAndCompute(self.template, None)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # 2. Rango de color Azul (ajustado según tu video)
        self.lower_blue = np.array([100, 150, 50])
        self.upper_blue = np.array([130, 255, 255])

        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.get_logger().info('Detector Híbrido Pro iniciado...')

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. Máscara de color con limpieza
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 1500:
                x, y, w, h = cv2.boundingRect(c)
                
                # Añadimos un pequeño margen (padding) para no cortar el logo
                pad = 15
                roi_gray = gray[max(0, y-pad):y+h+pad, max(0, x-pad):x+w+pad]
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2) # Amarillo: Color detectado

                # 2. Búsqueda de puntos clave ORB solo en el recorte
                kp_roi, des_roi = self.orb.detectAndCompute(roi_gray, None)
                
                if des_roi is not None:
                    matches = self.bf.match(self.des_ref, des_roi)
                    # Si hay suficientes puntos que coinciden (el logo está ahí)
                    if len(matches) > 15: # Umbral de confianza
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 4) # Verde: Identificado
                        cv2.putText(frame, f"INTEL DETECTADO ({len(matches)} pts)", (x, y-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Deteccion Pro", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(HybridVisionPro())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
