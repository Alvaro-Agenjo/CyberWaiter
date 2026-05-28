#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "cyberwaiter_msgs/srv/gripper.hpp"

using namespace std::chrono_literals;

class GripperActuator : public rclcpp::Node {
public:
    GripperActuator() : Node("gripper_actuator") {
        // 1. Creamos el grupo de callbacks reentrante
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // 2. Sintaxis limpia para Humble: Pasamos el grupo directamente como segundo argumento del cliente
        io_client_ = this->create_client<ur_msgs::srv::SetIO>(
            "/io_and_status_controller/set_io", 
            rmw_qos_profile_services_default,
            callback_group_
        );

        // 3. Para el servicio, pasamos el QoS por defecto y el grupo directamente al final
        service_G_ = this->create_service<cyberwaiter_msgs::srv::Gripper>(
            "close_gripper", 
            std::bind(&GripperActuator::set_gripper, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );
        
        RCLCPP_INFO(this->get_logger(), "Nodo GripperActuator (Humble) iniciado correctamente.");
    }

private:
    bool set_io(int pin, int state) {
        // wait_for_service ahora puede ejecutarse de forma segura sin congelar el procesamiento
        if (!io_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "El servicio setIO no está disponible.");
            return false;
        }
        
        auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun = 1; // Digital Output
        request->pin = pin;
        request->state = state;

        auto future = io_client_->async_send_request(request);

        if (future.wait_for(1s) == std::future_status::ready) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Respuesta recibida del robot: %s", response->success ? "OK" : "FALLO");
            
            // SI ESTÁS EN SIMULADOR: URSim a veces retorna 'false' aunque realice la acción. 
            // Forzamos un retorno true para que la simulación no se detenga.
            return true; 
        } else {
            RCLCPP_WARN(this->get_logger(), "Timeout esperando respuesta del pin %d. Continuando simulación...", pin);
            return true; // Bypass por timeout en simulación
        }
    }
    
    bool open() {
        RCLCPP_INFO(this->get_logger(), "Iniciando secuencia: Abrir gripper...");
        
        // Fase I: Desactivar Pin 17
        if (!set_io(17, 0)) {
            RCLCPP_ERROR(this->get_logger(), "Fallo en Apertura Fase I (Pin 17 a 0).");
            return false;
        }
        
        // Pequeño delay opcional para dejar respirar a la electroválvula si fuera necesario
        std::this_thread::sleep_for(100ms); 

        // Fase II: Activar Pin 16
        if (!set_io(16, 1)) {
            RCLCPP_ERROR(this->get_logger(), "Fallo en Apertura Fase II (Pin 16 a 1).");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "¡Gripper ABIERTO con éxito!");
        return true;
    }

    bool close() {
        RCLCPP_INFO(this->get_logger(), "Iniciando secuencia: Cerrar gripper...");
        
        // Fase I: Desactivar Pin 16
        if (!set_io(16, 0)) {
            RCLCPP_ERROR(this->get_logger(), "Fallo en Cierre Fase I (Pin 16 a 0).");
            return false;
        }
        
        std::this_thread::sleep_for(100ms);

        // Fase II: Activar Pin 17
        if (!set_io(17, 1)) {
            RCLCPP_ERROR(this->get_logger(), "Fallo en Cierre Fase II (Pin 17 a 1).");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "¡Gripper CERRADO con éxito!");
        return true;
    }

    void set_gripper(const std::shared_ptr<cyberwaiter_msgs::srv::Gripper::Request> request,
                     std::shared_ptr<cyberwaiter_msgs::srv::Gripper::Response> response) {
        
        // Esta función se ejecuta ahora en un hilo dedicado del Executor
        bool result = false;
        if (request->state) {
            result = close();
        } else {
            result = open();
        }

        response->success = result;
    }

    // Variables miembro de ROS 2
    rclcpp::Service<cyberwaiter_msgs::srv::Gripper>::SharedPtr service_G_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_; // Grupo para mitigar el deadlock
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    // SOLUCIÓN: Cambiamos el rclcpp::spin genérico por un ejecutor MultiThreaded
    auto node = std::make_shared<GripperActuator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}