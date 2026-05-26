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

#include "geometry_msgs/msg/pose.hpp"
// #include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
			subscriber_goal_orientation_ = this->create_subscription<geometry_msgs::msg::Vector3>("/deteccion/orientacion_final", 10,
				std::bind(&MisionController::GoalOrientationCallback, this, std::placeholders::_1));


			publisher_coordinates_ = this->create_publisher<geometry_msgs::msg::Pose>("coordinates", 10);
			subscriber_movement_state_ = this->create_subscription<std_msgs::msg::String>("movement_state", 10,
				std::bind(&MisionController::MovementCallback, this, std::placeholders::_1)); 

			
			_watchdog = this->create_wall_timer(TIMEOUT, std::bind(&MisionController::timeout, this));
			_watchdog->cancel();

			/*****************************************************************************************************************************
			 * ***************************************************************************************************************************
			 * bandeja_.x = 1; bandeja_.y = 1; bandeja_.z = 0;
			 * home_.x = 0; home_.y = 2; home_.z = 0;
			 * **************************************************************************************************************************/
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

			// if(msg->x == 0 && msg->y == 0 && msg->z == 0){
			// 	RCLCPP_INFO(this->get_logger(), "No se ha encontrado la bebida solicitada, saltando pedido");
			// 	logic(-1);
			// 	logic();
			// 	return;
			// }
			destino_.position.x = msg->x;
			destino_.position.y = msg->y;
			destino_.position.z = msg->z;

			logic();
		}
		void GoalOrientationCallback(const geometry_msgs::msg::Vector3::SharedPtr msg){
			if (estado_actual_ != Estado::IDENTIFICATION) return;

			tf2::Quaternion q;
			q.setRPY(rad2deg(msg->x), rad2deg(msg->y), rad2deg(msg->z));
			q.normalize();
			geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
			destino_.orientation = q_msg;

			logic();
		}
		void MovementCallback(const std_msgs::msg::String::SharedPtr msg){
			if(estado_actual_ == Estado::RETRIEVING_ITEM  && msg->data == "OK"){
				logic();
			}
			else if(estado_actual_ == Estado::DELIVERING_ITEM && msg->data == "OK"){
				logic();
			}
			else if(estado_actual_ == Estado::PROCESSING_ORDER &&  msg->data == "OK"){
				logic();
			}
			else{
				RCLCPP_INFO(this->get_logger(), "Estado de movimiento recibido: %s", msg->data.c_str());
				logic(-1);
			}
		}
		void logic( int results = 0){
			switch (estado_actual_){
			case Estado::IDLE:{
				RCLCPP_INFO(this->get_logger(), "Pedidio recibido"); 
				estado_actual_ = Estado::PROCESSING_ORDER;
				break;
			}
			case Estado::PROCESSING_ORDER:{
				//Hay mas latas que procesar
				if (counter_pedido == pedido.size()){

					counter_pedido = 0;
					//No
					auto state_msg = std_msgs::msg::String();
					state_msg.data = "Finish";
					RCLCPP_INFO(this->get_logger(), "Retire su pedido"); 
					publisher_state_->publish(state_msg);

					estado_actual_ = Estado::IDLE;
					break;
				}
				//SI 

				//Informamos a la GUI de que hemos cambiado de estado
				RCLCPP_INFO(this->get_logger(), "Procesando Pedido [ %d / %d ]", counter_pedido+1, (int)pedido.size()); 
				
				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Buscando item";
				RCLCPP_INFO(this->get_logger(), "Localizando Item"); 
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
				static int counter = 0;
				if (results == -1){
					estado_actual_ = Estado::PROCESSING_ORDER;
					logic();
					break;
				}

				if (counter < 1) {
					RCLCPP_INFO(this->get_logger(), "Esperando resultado de identificación...");
					counter++;
					_watchdog->cancel();
					return;
				}
				
				RCLCPP_INFO(this->get_logger(), "Objetivo localizado"); 
				counter = 0;
				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Recogiendo item";
				RCLCPP_INFO(this->get_logger(), "Recogiendo Item"); 
				publisher_state_->publish(state_msg);

				auto coord_msg = geometry_msgs::msg::Pose();
				coord_msg = destino_;
				publisher_coordinates_->publish(coord_msg);
				estado_actual_ = Estado::RETRIEVING_ITEM;
				break;
			}
			case Estado::RETRIEVING_ITEM:{
				if(results == -1) break;
				RCLCPP_INFO(this->get_logger(), "Desplazando a punto de recogida"); 

				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Entregando item";
				publisher_state_->publish(state_msg);

				auto coord_msg = geometry_msgs::msg::Pose();
				coord_msg = bandeja_;
				publisher_coordinates_->publish(coord_msg);
				estado_actual_ = Estado::DELIVERING_ITEM;
				break;
			}
			case Estado::DELIVERING_ITEM:{
				if(results == -1) break;
				RCLCPP_INFO(this->get_logger(), "Bebida servida"); 

				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Next";
				publisher_state_->publish(state_msg);

				auto coord_msg = geometry_msgs::msg::Pose();
				coord_msg = home_;
				publisher_coordinates_->publish(coord_msg);
				estado_actual_ = Estado::PROCESSING_ORDER;
				break;
			}
			default:
				break;
			}
		}
		void timeout(){
			RCLCPP_INFO(this->get_logger(), "No se ha encontrado la bebida solicitada, saltando pedido");
			_watchdog->cancel();

			auto state_msg = std_msgs::msg::String();
			state_msg.data = "No item";
			publisher_state_->publish(state_msg);

			logic(-1);
		}
	
	// void timer_callback()
	// 	{
	// 	auto message = std_msgs::msg::String();
	// 	message.data = "Buenos días señor, han pasado " + std::to_string(count_++) + " días desde su última visita";
	// 	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
	// 	publisher_->publish(message);
	// 	}



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
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_coordinates_;
		
		// Suscriptores
		rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscriber_GUI_;
		rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_goal_pose_;
		rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_goal_orientation_;
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

