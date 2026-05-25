import solara
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time

# --- ESTADO REACTIVO ---
robot_status_msg = solara.reactive("ESPERANDO CONEXIÓN...")
camera_image_data = solara.reactive(None)
cart = solara.reactive([])  # Lista de productos seleccionados
is_sending = solara.reactive(False)
selected_drink = solara.reactive("Coca Cola")
qty = solara.reactive(1)

# Diccionario para mapear nombres a IDs (debe coincidir con tu lógica de C++)
DRINKS = ["Coca Cola", "Fanta Limón"]
DRINK_TO_ID = {"Coca Cola": 0, "Fanta Limón": 1}

class SolaraRosNode(Node):
    def __init__(self):
        super().__init__('solara_interface_node')
        self.bridge = CvBridge()
        self.publisher_pedido = self.create_publisher(Int8MultiArray, 'pedido', 10)
        self.subscription_status = self.create_subscription(
            String, 'mision_state', self.status_callback, 10)

    def status_callback(self, msg):
        robot_status_msg.value = msg.data

    def enviar_pedido(self, lista_carrito):
        msg = Int8MultiArray()
        # Formato: [Total_latas, ID1, ID2, ...]
        total_latas = sum(item['qty'] for item in lista_carrito)
        data = [total_latas]
        
        for item in lista_carrito:
            drink_id = DRINK_TO_ID[item['drink']]
            for _ in range(item['qty']):
                data.append(drink_id)
        
        msg.data = data
        self.publisher_pedido.publish(msg)
        self.get_logger().info(f'Pedido enviado: {data}')

# --- INICIALIZACIÓN DE ROS ---
ros_node = None
def start_ros():
    global ros_node
    if not rclpy.ok():
        rclpy.init()
    ros_node = SolaraRosNode()
    rclpy.spin(ros_node)

threading.Thread(target=start_ros, daemon=True).start()

# --- LÓGICA DE LA INTERFAZ ---
def add_to_cart():
    # Añade la bebida actual al carrito
    current_cart = list(cart.value)
    current_cart.append({"drink": selected_drink.value, "qty": qty.value})
    cart.value = current_cart

def send_to_robot():
    if ros_node and cart.value:
        is_sending.value = True
        ros_node.enviar_pedido(cart.value) # Corregido el nombre del método
        time.sleep(2) # Simulación de envío
        cart.value = [] # Limpiar carrito tras enviar
        is_sending.value = False

# [El resto de tu componente Page y CSS se mantiene igual...]
css = """
@import url('https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&display=swap');

/* AGGRESSIVE RESET to kill any white background */
html, body, #app, .v-application, .v-application--wrap, .solara-container, .solara-autoroute-content, main, .v-main {
    background-color: #0f172a !important;
    background-image: radial-gradient(circle at 50% -20%, #1e293b, #0f172a) !important;
    color: #f1f5f9 !important;
    box-shadow: none !important;
}

/* Ensure no white shadows or borders on the main container */
.v-sheet, .v-card {
    background-color: transparent !important;
    box-shadow: none !important;
}

.solara-container {
    font-family: 'Outfit', sans-serif !important;
    min-height: 100vh;
    width: 100%;
    margin: 0;
    padding: 2rem;
}

.cyber-header {
    font-size: 2.5rem;
    font-weight: 800;
    background: linear-gradient(to right, #60a5fa, #a78bfa);
    -webkit-background-clip: text;
    background-clip: text;
    -webkit-text-fill-color: transparent;
    margin: 0;
}

.status-badge {
    background: rgba(59, 130, 246, 0.2) !important;
    border: 1px solid rgba(59, 130, 246, 0.5);
    /*color: #60a5fa;*/
    color: white !important;
    padding: 0.5rem 1rem;
    border-radius: 99rem;
    font-weight: 600;
    display: inline-flex;
    align-items: center;
    gap: 0.5rem;
}

.status-dot {
    width: 8px;
    height: 8px;
    background: #60a5fa;
    border-radius: 50%;
    box-shadow: 0 0 10px #60a5fa;
    animation: pulse 2s infinite;
}

@keyframes pulse {
    0% { opacity: 1; transform: scale(1); }
    50% { opacity: 0.5; transform: scale(1.2); }
    100% { opacity: 1; transform: scale(1); }
}

.camera-viewport {
    width: 100%;
    aspect-ratio: 16/9;
    background: #000;
    border-radius: 1rem;
    overflow: hidden;
    position: relative;
    border: 2px solid #334155;
    display: flex;
}

.camera-viewport img {
    width: 100%;
    height: 100%;
    object-fit: cover;
    opacity: 0.8;
    filter: contrast(1.1) brightness(0.9);
}

.panel {
    background: rgba(30, 41, 59, 0.7) !important;
    backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px);
    border: 1px solid rgba(255, 255, 255, 0.1) !important;
    border-radius: 1.5rem;
    padding: 2rem;
    box-shadow: 0 10px 30px -10px rgba(0, 0, 0, 0.5);
    height: 100%;
    display: flex;
    flex-direction: column;
}

.panel h2 {
    font-size: 1.5rem !important;
    font-weight: 700 !important;
    color: #f1f5f9 !important;
    margin-bottom: 1.5rem !important;
    letter-spacing: -0.02em !important;
}

.spec-card-styled {
    background: rgba(59, 130, 246, 0.1) !important;
    padding: 1rem;
    border-radius: 0.75rem;
    border: 1px solid rgba(255, 255, 255, 0.1) !important;
    text-align: center;
    flex: 1;
}

.order-item-styled {
    background: rgba(255, 255, 255, 0.03) !important;
    padding: 1rem;
    border-radius: 0.75rem;
    border: 1px solid rgba(255, 255, 255, 0.05) !important;
    display: flex;
    justify-content: space-between;
    align-items: center;
}

/* Specific text colors */
.text-main {
    color: #f1f5f9 !important;
}

.text-dim {
    color: #94a3b8 !important;
}

.label-text {
    color: #94a3b8 !important;
    font-size: 0.875rem !important;
    font-weight: 600 !important;
    text-transform: uppercase !important;
    letter-spacing: 0.05em !important;
    margin-bottom: 0.5rem !important;
}

/* Forzando estilos en botones de Vuetify */
.v-btn.btn-add {
    background: rgba(255, 255, 255, 0.05) !important;
    border: 1px solid rgba(255, 255, 255, 0.1) !important;
    color: white !important;
    padding: 1rem !important;
    border-radius: 0.75rem !important;
    font-family: 'Outfit', sans-serif !important;
    font-weight: 600 !important;
    letter-spacing: 0 !important;
    height: auto !important;
    text-transform: none !important;
    transition: all 0.2s !important;
    width: 100% !important;
    margin-top: 1rem !important;
    margin-bottom: 1rem !important;
    box-shadow: none !important;
}

.v-btn.btn-add:hover {
    background: rgba(59, 130, 246, 0.1) !important;
    border-color: #3b82f6 !important;
}

.v-btn.btn-send {
    background: linear-gradient(135deg, #3b82f6, #8b5cf6) !important;
    color: white !important;
    border: none !important;
    padding: 1.25rem !important;
    border-radius: 0.75rem !important;
    font-family: 'Outfit', sans-serif !important;
    font-weight: 800 !important;
    font-size: 1.1rem !important;
    height: auto !important;
    box-shadow: 0 4px 15px rgba(59, 130, 246, 0.3) !important;
    transition: transform 0.2s, box-shadow 0.2s !important;
    margin-top: auto !important;
    width: 100% !important;
}

.v-btn.btn-send:hover {
    transform: translateY(-2px) !important;
    box-shadow: 0 6px 20px rgba(59, 130, 246, 0.4) !important;
}

.v-btn.btn-send[disabled] {
    background: #475569 !important;
    box-shadow: none !important;
    opacity: 0.5 !important;
    transform: none !important;
}

/* --- Estilos para Dropdown e Inputs --- */

/* Forzar que CUALQUIER elemento dentro de nuestros selectores e inputs sea blanco */
.custom-dropdown, .custom-dropdown *, 
.custom-input, .custom-input * {
    color: white !important;
}

/* Forzar el fondo del menú desplegable (en Vuetify 3 usan v-overlay-container, en Vuetify 2 v-menu__content) */
.v-overlay-container .v-list,
.v-overlay-container .v-sheet,
.v-menu__content,
.v-menu__content .v-list {
    background-color: #475569 !important;
}

/* Forzar que el texto dentro del menú desplegable sea blanco */
.v-overlay-container *,
.v-menu__content * {
    color: white !important;
}

/* Efecto hover en el menú */
.v-list-item:hover, .v-list-item--active {
    background-color: #3b82f6 !important;
}
"""

@solara.component
def Page():
    solara.Style(css)
    
    with solara.Column(classes=["solara-container"], style={"padding": "2rem"}):
        with solara.Column(style={"max-width": "1200px", "margin": "0 auto", "width": "100%"}):
            
            # --- Header ---
            with solara.Row(justify="space-between", style={"width": "100%", "margin-bottom": "2rem", "align-items": "center"}):
                solara.HTML(tag="h1", unsafe_innerHTML="CyberWaiter", classes=["cyber-header"])
                with solara.Row(classes=["status-badge"], style={"padding": "0.5rem 1.25rem"}):
                    solara.HTML(tag="div", unsafe_innerHTML="<div class='status-dot'></div>")
                    solara.Text(robot_status_msg.value, style={"font-size": "0.85rem", "letter-spacing": "0.05em", "font-weight": "700"})
                    
            with solara.Row(style={"gap": "2rem", "flex-wrap": "wrap"}):
                
                # --- Panel Izquierdo ---
                with solara.Column(style={"flex": "2", "min-width": "300px"}):
                    with solara.Column(classes=["panel"]):
                        with solara.Row(classes=["camera-viewport"]):
                            if camera_image_data.value:
                                solara.HTML(tag="img", attributes={"src": camera_image_data.value, "alt": "Robot Stream"})
                            else:
                                solara.HTML(tag="img", attributes={"src": "https://plus.unsplash.com/premium_photo-1683120912290-798782bb446f?q=80&w=2070&auto=format&fit=crop", "alt": "Robot View"})
                        
                        with solara.Row(style={"margin-top": "1.5rem", "gap": "1rem"}):
                            with solara.Column(classes=["spec-card-styled"]):
                                solara.Text("ESTADO", classes=["text-dim"], style={"font-size": "0.75rem", "margin-bottom": "0.25rem"})
                                solara.Text(robot_status_msg.value, classes=["text-main"], style={"font-weight": "800", "font-size": "1.1rem"})
                            with solara.Column(classes=["spec-card-styled"]):
                                solara.Text("POSICIÓN", classes=["text-dim"], style={"font-size": "0.75rem", "margin-bottom": "0.25rem"})
                                solara.Text("HOME", classes=["text-main"], style={"font-weight": "800", "font-size": "1.1rem"})

                # --- Panel Derecho ---
                with solara.Column(style={"flex": "1", "min-width": "300px"}):
                    with solara.Column(classes=["panel"]):
                        solara.HTML(tag="h2", unsafe_innerHTML="HACER PEDIDO")
                        
                        with solara.Column(style={"margin-bottom": "1.5rem"}):
                            solara.HTML(tag="label", unsafe_innerHTML="SELECCIONAR BEBIDA", classes=["label-text"])
                            solara.Select(label="", values=DRINKS, value=selected_drink, classes=["custom-dropdown"])
                        
                        with solara.Column(style={"margin-bottom": "1rem"}):
                            solara.HTML(tag="label", unsafe_innerHTML="CANTIDAD", classes=["label-text"])
                            solara.InputInt(label="", value=qty, classes=["custom-input"])
                        
                        solara.Button("Añadir al carrito", on_click=add_to_cart, classes=["btn-add"])
                        
                        with solara.Column(style={"flex": "1", "margin-top": "1rem", "overflow-y": "auto", "gap": "0.75rem", "margin-bottom": "1rem"}):
                            for item in cart.value:
                                with solara.Row(classes=["order-item-styled"]):
                                    solara.Text(item["drink"],style={"color": "white", "font-weight": "800"})
                                    solara.Text(f"x{item['qty']}", style={"color": "white", "font-weight": "800"})
                        
                        is_disabled = len(cart.value) == 0 or is_sending.value
                        solara.Button(
                            "MANDAR AL ROBOT" if not is_sending.value else "ROBOT EN MOVIMIENTO",
                            on_click=send_to_robot,
                            disabled=is_disabled,
                            classes=["btn-send"]
                        )
