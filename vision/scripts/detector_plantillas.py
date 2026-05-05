import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class TemplateDetector(Node):
    def __init__(self):
        super().__init__('template_detector_node')
        self.bridge = CvBridge()
        
        # 1. Cargamos la plantilla de referencia completa (imagen_7.png)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        base_path = os.path.abspath(os.path.join(script_dir, '..'))
        ruta_imagen = os.path.join(base_path, 'reference.jpg')

        self.template = cv2.imread(ruta_imagen, 0)

        if self.template is None:
            self.get_logger().error('¡No encuentro el archivo /home/romin/LAyR/reference.jpg!')
            return
        
        # Guardamos dimensiones originales de la plantilla
        (self.tH, self.tW) = self.template.shape[:2]

        # 2. Configuración del Detector y NMS
        # Umbral de coincidencia (0.0 a 1.0). 0.8 es un buen equilibrio.
        self.threshold = 0.8 
        # NMS para limpiar recuadros múltiples (cuanto más bajo, más estricto)
        self.nms_threshold = 0.3 

        self.img_sub = self.create_subscription(
            Image, 
            '/camera/color/image_raw', 
            self.color_callback, 
            10)
        
        self.get_logger().info(f'Buscando plantilla completa de {self.tW}x{self.tH}...')

    # Función auxiliar para Non-Maximum Suppression (NMS)
    def non_max_suppression_fast(self, boxes, overlapThresh):
        if len(boxes) == 0:
            return []
        if boxes.dtype.kind == "i":
            boxes = boxes.astype("float")
        pick = []
        x1 = boxes[:,0]; y1 = boxes[:,1]
        x2 = boxes[:,2]; y2 = boxes[:,3]
        scores = boxes[:,4]
        area = (x2 - x1 + 1) * (y2 - y1 + 1)
        idxs = np.argsort(scores)
        while len(idxs) > 0:
            last = len(idxs) - 1
            i = idxs[last]; pick.append(i)
            xx1 = np.maximum(x1[i], x1[idxs[:last]])
            yy1 = np.maximum(y1[i], y1[idxs[:last]])
            xx2 = np.minimum(x2[i], x2[idxs[:last]])
            yy2 = np.minimum(y2[i], y2[idxs[:last]])
            w = np.maximum(0, xx2 - xx1 + 1)
            h = np.maximum(0, yy2 - yy1 + 1)
            overlap = (w * h) / area[idxs[:last]]
            idxs = np.delete(idxs, np.concatenate(([last], np.where(overlap > overlapThresh)[0])))
        return boxes[pick].astype("int")

    def color_callback(self, data):
        # 1. Preparar la imagen
        frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = [] # Lista para guardar candidatos

        # 2. BUCLE MULTI-ESCALA: Buscamos la plantilla en varios tamaños
        # Recorremos escalas desde 50% hasta 150% del tamaño de la plantilla
        for scale in np.linspace(0.5, 1.5, 10):
            # Redimensionar la plantilla
            resized_template = cv2.resize(self.template, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            (rW, rH) = resized_template.shape

            # Si la plantilla redimensionada es más grande que el frame, la saltamos
            if gray_frame.shape[0] < rW or gray_frame.shape[1] < rH:
                continue

            # Ejecutar Coincidencia de Plantillas Normalizada
            result = cv2.matchTemplate(gray_frame, resized_template, cv2.TM_CCOEFF_NORMED)
            
            # Encontrar todas las ubicaciones que superan el threshold
            (y_locs, x_locs) = np.where(result >= self.threshold)
            scores = result[y_locs, x_locs]

            # Añadir cada candidato a la lista
            for (x, y, score) in zip(x_locs, y_locs, scores):
                # Guardar [x1, y1, x2, y2, score]
                detections.append([x, y, x + rW, y + rH, score])

        # 3. Aplicar Non-Maximum Suppression (NMS) para limpiar duplicados
        boxes = np.array(detections)
        if len(boxes) > 0:
            # self.get_logger().info(f'Candidatos crudos: {len(boxes)}')
            picked_boxes = self.non_max_suppression_fast(boxes, self.nms_threshold)
            self.get_logger().info(f'Plantillas detectadas tras NMS: {len(picked_boxes)}')

            # 4. Dibujar los resultados finales
            for (x1, y1, x2, y2, score) in picked_boxes:
                # Dibujar recuadro verde
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                # Escribir puntuación (para debug)
                cv2.putText(frame, f"Match: {score:.2f}", (x1, y1 - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Deteccion Inteligente de Plantillas", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TemplateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
