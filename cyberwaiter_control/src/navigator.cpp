#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cyberwaiter_msgs/msg/movement.hpp"

#include "kdl/frames.hpp"

#include "cyberwaiter_msgs/srv/gripper.hpp"

#define plane_h 0.17

using namespace std::chrono_literals;

class Navigator : public rclcpp::Node {
public:
    
    Navigator() : Node("navigator") {
        rclcpp::QoS best_effort__Volatile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        rclcpp::QoS reliable__Volatile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        rclcpp::QoS transient_local = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        

        // Publica el punto a alcanzar
        coordinate_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/goals/goal_coord", reliable__Volatile);
        
        // Comunica a mision el correcto movimiento a punto final
        state_pub_ = this->create_publisher<std_msgs::msg::String>("/movement_state", reliable__Volatile);


        // Recive el movimiento a realizar y el punto a alcanzar
        command_sub_ = this->create_subscription<cyberwaiter_msgs::msg::Movement>("/movimiento", reliable__Volatile, 
            std::bind(&Navigator::command_callback, this, std::placeholders::_1));
        
        // Obtienen las coordenadas del TCP 
        tcp_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/tcp_pose_broadcaster/pose", reliable__Volatile, 
            std::bind(&Navigator::tcp_callback, this, std::placeholders::_1));

        // Recibe confirmacion de objetivo alcanzado por parte de kin_control
        kin_report_sub_ = this->create_subscription<std_msgs::msg::String>("/goals/state", reliable__Volatile, 
            std::bind(&Navigator::kin_report_callback, this, std::placeholders::_1));
        
        
        /* Services */
        callback_group_client_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        gripper_client_ = this->create_client<cyberwaiter_msgs::srv::Gripper>(
            "/close_gripper", 
            rmw_qos_profile_services_default, 
            callback_group_client_);
       
    }

private:
    void tcp_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        TCP_.M = KDL::Rotation::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        TCP_.p = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    }
    void kin_report_callback(const std_msgs::msg::String::SharedPtr msg){

        //std::this_thread::sleep_for(std::chrono::milliseconds(2000)); 
        static bool ignore_next = false;
        if(msg->data == "NOK"){
            RCLCPP_ERROR(this->get_logger(), "[Navigator] respuesta del solver 'FALLO', pose no alcanzada.");
            goals.clear(); 
            return;
        }
        else if (msg->data == "RETRY"){
            ignore_next = true;
            goals.clear();
            RCLCPP_ERROR(this->get_logger(), "[Navigator] respuesta del solver 'RETRY', reintentando desde escape...");
            state_pub_->publish(std_msgs::msg::String().set__data("RETRY"));
            return;
        }
        else if(msg->data == "OK"){
            if (ignore_next) {
                RCLCPP_ERROR(this->get_logger(), "[Navigator] [DEBUG] ignoring escape ok.");
                ignore_next = false;
                return;
            }
            RCLCPP_INFO(this->get_logger(), "[Navigator] respuesta del solver 'COMPLETADO', pose alcanzada.");
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Navigator] Mensaje de estado desconocido: %s", msg->data.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "[Navigator] Puntos restantes: %d", (int)goals.size());
        
        // Es el ultimo punto de la secuencia
        if(goals.size() == 0){
            RCLCPP_INFO(this->get_logger(), "[Navigator] Secuencia completada, notificando");
            state_pub_->publish(std_msgs::msg::String().set__data("OK"));

            if (gripper != 0){
                RCLCPP_INFO(this->get_logger(), "[Navigator] Ejecutando acción de pinza: %d", gripper);
                set_gripper(gripper == 1? true : false);                
            }
        }
        else{
            RCLCPP_INFO(this->get_logger(), "[Navigator] Enviando siguiente waypoint (Quedan %d en cola)...", (int)goals.size());
            send_goal(goals.front());
        }
        
    }
    void command_callback(const cyberwaiter_msgs::msg::Movement::SharedPtr msg) {
        
        if (goals.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "Acceso denegado, tarea en proceso ...");
            return; //Ignorar comando si se está ejecutando otro movimiento
        }

        gripper = msg->gripper_close;
        KDL::Frame goal;
        goal.M = KDL::Rotation::Quaternion(msg->point.orientation.x, msg->point.orientation.y, msg->point.orientation.z, msg->point.orientation.w);
        goal.p = KDL::Vector(msg->point.position.x, msg->point.position.y, msg->point.position.z);

        //Analisis de comando recibido
        if(msg->modo.data == "TO_POINT"){
            RCLCPP_INFO(this->get_logger(), "[Navigator]Comando recibido: TO_POINT");
            coordinate_pub_->publish(msg->point);
        }
        else if(msg->modo.data == "APROX"){
            RCLCPP_INFO(this->get_logger(), "[Navigator]Comando recibido: APROX");
            
            goals.push_back(goal);
            goals.push_front(goals.back() * KDL::Frame(KDL::Vector(0.0, 0, -0.1))); // Punto de aproximación a 10cm del objetivo 


            RCLCPP_INFO(this->get_logger(), "[Navigator] Pt aproximación: x: %f, y: %f, z: %f", goals.front().p.x(), goals.front().p.y(), goals.front().p.z());
            RCLCPP_INFO(this->get_logger(), "[Navigator] Pt objetivo: x: %f, y: %f, z: %f", goals.back().p.x(), goals.back().p.y(), goals.back().p.z());

            send_goal(goals.front());
        }
        else if(msg->modo.data == "DESPLAZAMIENTO"){

            RCLCPP_INFO(this->get_logger(), "[Navigator] Comando recibido: DESPLAZAMIENTO");
            goals.push_back(goal);
            
            goals.push_front(KDL::Frame(KDL::Vector(0.0, 0.0, plane_h)) * goal);
            goals.push_front(KDL::Frame(KDL::Vector(0.0, 0.0, plane_h)) * TCP_);

            send_goal(goals.front());
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "[Navigator] Comando no reconocido: %s", msg->modo.data.c_str());
            return;
        }
    }
    
    void send_goal(const KDL::Frame & goal){

        RCLCPP_INFO(this->get_logger(), "[Navigator] Enviando objetivo a kin_control..., x: %f, y: %f, z: %f", goal.p.x(), goal.p.y(), goal.p.z());

        geometry_msgs::msg::Pose goal_msg;
        goal_msg.position.x = goal.p.x();
        goal_msg.position.y = goal.p.y();
        goal_msg.position.z = goal.p.z();
        goal.M.GetQuaternion(goal_msg.orientation.x, goal_msg.orientation.y, goal_msg.orientation.z, goal_msg.orientation.w);

        // goal_msg.orientation.x = goal.M.x();
        // goal_msg.orientation.y = goal.M.y();
        // goal_msg.orientation.z = goal.M.z();
        // goal_msg.orientation.w = goal.M.w();
        coordinate_pub_->publish(goal_msg);

        goals.pop_front();
    }
    
    bool set_gripper(bool close){
        auto request = std::make_shared<cyberwaiter_msgs::srv::Gripper::Request>();
        request->state = close;

        while (!gripper_client_->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "El servicio de gripper no está disponible.");
            return false;
        }

        auto future = gripper_client_->async_send_request(request);
        
        auto response = future.get();
        return response->success;

    }

    int idle = true;
    int gripper = 0; //1: cerrado, -1: abierto, 0: no action
    KDL::Frame TCP_;
    std::deque<KDL::Frame> goals;

    rclcpp::Subscription<cyberwaiter_msgs::msg::Movement>::SharedPtr command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kin_report_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr coordinate_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

    rclcpp::Client<cyberwaiter_msgs::srv::Gripper>::SharedPtr gripper_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_client_;
      
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Navigator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}