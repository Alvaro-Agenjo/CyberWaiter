import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorTuner(Node):
    def __init__(self):
        super().__init__('color_tuner_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        
        # Crear ventana y barras de control
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L-H", "Trackbars", 0, 179, self.nothing)
        cv2.createTrackbar("L-S", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("L-V", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("U-H", "Trackbars", 179, 179, self.nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, self.nothing)
        cv2.createTrackbar("U-V", "Trackbars", 255, 255, self.nothing)

    def nothing(self, x):
        pass

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Leer valores de las barras
        l_h = cv2.getTrackbarPos("L-H", "Trackbars")
        l_s = cv2.getTrackbarPos("L-S", "Trackbars")
        l_v = cv2.getTrackbarPos("L-V", "Trackbars")
        u_h = cv2.getTrackbarPos("U-H", "Trackbars")
        u_s = cv2.getTrackbarPos("U-S", "Trackbars")
        u_v = cv2.getTrackbarPos("U-V", "Trackbars")

        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])

        # Crear máscara y aplicar
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("Original", frame)
        cv2.imshow("Mascara (Blanco = Detectado)", mask)
        cv2.imshow("Resultado Filtrado", result)
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(ColorTuner())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
