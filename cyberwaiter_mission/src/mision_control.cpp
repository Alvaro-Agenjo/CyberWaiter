/**********************************************************************
 * Recepcion de pedido desde GUI check
 * Envio de referencia de identificacion a reconocimiento check
 * Recepcion de coordenadas del item desde reconocimiento check
 * Envio de coordenadas a control cinemático check
 * Recepcion de estado de movimiento desde control cinemático check
 * Envio de coordenadas (bandeja) ----
 * Envio de coordenadas (home) ----
 *********************************************************************/

// general lib
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <list>

// Ros related libs
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

// #include "geometry_msgs/msg/pose.hpp"
// #include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "cyberwaiter_msgs/msg/movement.hpp"

//Own libs
#include "Types.h"


using namespace std::chrono_literals;
// using std::placeholders::_1;

#define TIMEOUT 10s
#define rad2deg(x) ((x)*180.0/M_PI)

class MisionController : public rclcpp::Node{
    // QoS

	public:
		MisionController() : Node("MisionController"){
			publisher_state_ = this->create_publisher<std_msgs::msg::String>("mision_state",10);
			subscriber_GUI_ = this->create_subscription<std_msgs::msg::Int8MultiArray>("pedido", 10,
				std::bind(&MisionController::PedidoCallback, this, std::placeholders::_1)); 
			

			publisher_identification_reference_ = this->create_publisher<std_msgs::msg::String>("identification_reference", 10);
			subscriber_goal_pose_ = this->create_subscription<geometry_msgs::msg::Vector3>("/deteccion/posicion_final", 10,
				std::bind(&MisionController::GoalPoseCallback, this, std::placeholders::_1));
			// subscriber_goal_orientation_ = this->create_subscription<geometry_msgs::msg::Vector3>("/deteccion/orientacion_final", 10,
			// 	std::bind(&MisionController::GoalOrientationCallback, this, std::placeholders::_1));


			publisher_coordinates_ = this->create_publisher<cyberwaiter_msgs::msg::Movement>("/movimiento", 10);
			subscriber_movement_state_ = this->create_subscription<std_msgs::msg::String>("/movement_state", 10,
				std::bind(&MisionController::MovementCallback, this, std::placeholders::_1)); 

			
			_watchdog = this->create_wall_timer(TIMEOUT, std::bind(&MisionController::timeout, this));
			_watchdog->cancel();

			bandeja_.position.x =  0.21087346633307502; bandeja_.position.y = -0.45922171813440504; bandeja_.position.z = 0.06746603759115141;
			bandeja_.orientation.x = -0.5587809221483487; bandeja_.orientation.y = 0.45135463268752124; bandeja_.orientation.z = -0.5991390364746974; bandeja_.orientation.w = 0.3536598529190517;
			
			home_.position.x = -0.2790011187404149; home_.position.y = -0.1738465428383369; home_.position.z = 0.17388733559498376;
			home_.orientation.x = -0.26994772496344116; home_.orientation.y = 0.6680276858064119; home_.orientation.z = -0.6933493761679329; home_.orientation.w = 0.011570624474531195;
			
		}

	private:
		void PedidoCallback(const std_msgs::msg::Int8MultiArray::SharedPtr msg){
			
			if(estado_actual_ != Estado::IDLE) return; //Solo procesamos pedidos si no estamos preparando otro
			if(msg->data[0] == 0) {
				RCLCPP_WARN(this->get_logger(), "[WARN] se ha enviado un pedido vacio");
				return; 
			}
			//Rellenamos el pedido
			pedido.resize(msg->data[0]);
			for(int n = 0; n < msg->data[0]; n++)
				pedido[n] = msg->data[1+n];
			
			// Informamos de la llegada del mensaje
			logic();
			logic();
			
		}
		void GoalPoseCallback(const geometry_msgs::msg::Vector3::SharedPtr msg){
			if (estado_actual_ != Estado::IDENTIFICATION) return;
			RCLCPP_INFO(this->get_logger(), "[Mision Control] Coordenadas de objetivo recibidas");
			
			destino_.position.x = msg->x;
			destino_.position.y = msg->y;
			destino_.position.z = msg->z;

			logic();
		}
		// void GoalOrientationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg){
		// 	if (estado_actual_ != Estado::IDENTIFICATION) return;

		// 	tf2::Quaternion q;
		// 	q.setRPY(rad2deg(msg->x), rad2deg(msg->y), rad2deg(msg->z));
		// 	q.normalize();
		// 	geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
		// 	destino_.orientation = q_msg;

		// 	logic();
		// }
		void MovementCallback(const std_msgs::msg::String::SharedPtr msg){
			if (msg->data == "OK"){
				RCLCPP_INFO(this->get_logger(), "[Mision Control] Movimiento completado con exito");
				logic();
			}
			else if (msg->data == "NOK"){
				RCLCPP_ERROR(this->get_logger(), "[Mision Control] Movimiento fallido, abortando ...");
				logic(-1);
			}
			else if (msg->data == "RETRY"){
				RCLCPP_INFO(this->get_logger(), "[Mision Control] Movimiento fallido reintentando...");
				switch (estado_actual_){
					case Estado::RETRIEVING_ITEM:{
						estado_actual_ = Estado::IDENTIFICATION;
						break;
					}
					case Estado::DELIVERING_ITEM:{
						estado_actual_ = Estado::RETRIEVING_ITEM;
						break;
					}
					case Estado::PROCESSING_ORDER:{
						estado_actual_ = Estado::DELIVERING_ITEM;
						break;
					}
				}
				logic();
			}
		}

		void logic( int results = 0){
			switch (estado_actual_){
			case Estado::IDLE:{
				RCLCPP_INFO(this->get_logger(), "[Mision Control] Pedido recibido"); 
				estado_actual_ = Estado::PROCESSING_ORDER;
				break;
			}
			case Estado::PROCESSING_ORDER:{
				RCLCPP_INFO(this->get_logger(), "[Mision Control] PROCESING ORDER...");
				//Hay mas latas que procesar
				if (counter_pedido == pedido.size()){

					counter_pedido = 0;
					//No
					auto state_msg = std_msgs::msg::String();
					state_msg.data = "Finish";
					RCLCPP_INFO(this->get_logger(), "[Mision Control] Pedido completado. Retire su pedido"); 
					publisher_state_->publish(state_msg);

					estado_actual_ = Estado::IDLE;
					break;
				}
				//SI 

				//Informamos a la GUI de que hemos cambiado de estado
				RCLCPP_INFO(this->get_logger(), "[Mision Control] Procesando Pedido [ %d / %d ]", counter_pedido+1, (int)pedido.size()); 
				
				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Buscando item";
				RCLCPP_INFO(this->get_logger(), "[Mision Control] Localizando Item"); 
				publisher_state_->publish(state_msg);
				
				//Envio dato a reconocimiento
				auto id_msg = std_msgs::msg::String();
				switch(pedido[counter_pedido]){
					case 0:
						id_msg.data = "COCA-COLA";
						break;
					case 1:
						id_msg.data = "FANTA-NARANJA";
						break;
					case 2:
						id_msg.data = "Agua";
						break;
					case 3:
						id_msg.data = "FANTA-LIMON";
						break;
					case 4:
						id_msg.data = "CERVEZA";
						break;
				}
				publisher_identification_reference_->publish(id_msg);
				counter_pedido++;
				estado_actual_ = Estado::IDENTIFICATION;

				_watchdog->reset();
				break;
			}
			case Estado::IDENTIFICATION:{
				RCLCPP_INFO(this->get_logger(), "[Mision Control] IDENTIFICACION ...");
				if (results == -1){
					RCLCPP_WARN(this->get_logger(), "[Mision Control] Fallo en identificación. Pasando al siguiente elemento del pedido.");					estado_actual_ = Estado::PROCESSING_ORDER;
					logic();
					break;
				}
				
				_watchdog->cancel();
				RCLCPP_INFO(this->get_logger(), "[Mision Control] Objetivo localizado"); 
				
				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Recogiendo item";
				RCLCPP_INFO(this->get_logger(), "[Mision Control] Recogiendo Item"); 
				publisher_state_->publish(state_msg);

				auto coord_msg = cyberwaiter_msgs::msg::Movement();
				coord_msg.point = destino_;
				coord_msg.modo.data = "APROX";
				coord_msg.gripper_close = 1;
				publisher_coordinates_->publish(coord_msg);
				estado_actual_ = Estado::RETRIEVING_ITEM;
				break;
			}
			case Estado::RETRIEVING_ITEM:{
				RCLCPP_INFO(this->get_logger(), "[Mision Control] RETRIEVING ITEM ...");
				if(results == -1) break;
				RCLCPP_INFO(this->get_logger(), "[Mision Control] Desplazando a punto de recogida"); 

				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Entregando item";
				publisher_state_->publish(state_msg);

				auto coord_msg = cyberwaiter_msgs::msg::Movement();
				coord_msg.point = bandeja_;

				coord_msg.point.position.y = bandeja_.position.y - 0.1 * (counter_pedido - 1);
				coord_msg.modo.data = "DESPLAZAMIENTO";
				coord_msg.gripper_close = -1;
				publisher_coordinates_->publish(coord_msg);
				estado_actual_ = Estado::DELIVERING_ITEM;
				break;
			}
			case Estado::DELIVERING_ITEM:{
				RCLCPP_INFO(this->get_logger(), "[Mision Control] DELIVERING ITEM ...");
				if(results == -1) break;
				RCLCPP_INFO(this->get_logger(), "Bebida servida"); 

				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Next";
				publisher_state_->publish(state_msg);

				auto coord_msg = cyberwaiter_msgs::msg::Movement();
				coord_msg.point = home_;
				coord_msg.modo.data = "TO_POINT";
				coord_msg.gripper_close = 0;
				publisher_coordinates_->publish(coord_msg);
				estado_actual_ = Estado::PROCESSING_ORDER;
				break;
			}
			default:
				break;
			}
		}
		void timeout(){
			RCLCPP_INFO(this->get_logger(), "[Mision Control] Limite de tiempo alcanzado, deteccion dada por erronea");
			_watchdog->cancel();

			auto state_msg = std_msgs::msg::String();
			state_msg.data = "No item";
			publisher_state_->publish(state_msg);

			logic(-1);
		}
	
		// Otras variables miembro
		Estado estado_actual_ = Estado::IDLE;
		std::vector<int> pedido;
		int counter_pedido = 0;

		// geometry_msgs::msg::Pose destino_;
		geometry_msgs::msg::Pose destino_;
		geometry_msgs::msg::Pose bandeja_;
		geometry_msgs::msg::Pose home_;
		
		// Publicadores
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_state_;
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_identification_reference_;
		rclcpp::Publisher<cyberwaiter_msgs::msg::Movement>::SharedPtr publisher_coordinates_;
		
		// Suscriptores
		rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscriber_GUI_;
		rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_goal_pose_;
		// rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_goal_orientation_;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_movement_state_;
		
		// Servicios

		
		// Timers
		rclcpp::TimerBase::SharedPtr _watchdog;
		
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MisionController>());
	rclcpp::shutdown();
	return 0;
}

