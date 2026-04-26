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
#include "geometry_msgs/msg/point.hpp"


//Own libs
#include "Types.h"


using namespace std::chrono_literals;
// using std::placeholders::_1;

class MisionController : public rclcpp::Node{
    // QoS

	public:
		MisionController() : Node("MisionController"){
			publisher_state_ = this->create_publisher<std_msgs::msg::String>("mision_state",10);

			subscriber_GUI_ = this->create_subscription<std_msgs::msg::Int8MultiArray>("pedido", 10,
				std::bind(&MisionController::PedidoCallback, this, std::placeholders::_1)); 
			
			publisher_identification_reference_ = this->create_publisher<std_msgs::msg::Int8>("identification_reference", 10);

			subscriber_ID_ = this->create_subscription<geometry_msgs::msg::Point>("identification_coordinates", 10,
				std::bind(&MisionController::IdentificationCallback, this, std::placeholders::_1));
			
			publisher_coordinates_ = this->create_publisher<geometry_msgs::msg::Point>("coordinates", 10);

			subscriber_movement_state_ = this->create_subscription<std_msgs::msg::String>("movement_state", 10,
				std::bind(&MisionController::MovementCallback, this, std::placeholders::_1)); 

			bandeja_.x = 1; bandeja_.y = 1; bandeja_.z = 0;
			home_.x = 0; home_.y = 2; home_.z = 0;
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
		void IdentificationCallback(const geometry_msgs::msg::Point::SharedPtr msg){
			if (estado_actual_ != Estado::IDENTIFICATION) return;

			if(msg->x == 0 && msg->y == 0 && msg->z == 0){
				RCLCPP_INFO(this->get_logger(), "No se ha encontrado la bebida solicitada, saltando pedido");
				logic(-1);
				logic();
				return;
			}
			destino_.x = msg->x;
			destino_.y = msg->y;
			destino_.z = msg->z;

			logic();
		}
		void MovementCallback(const std_msgs::msg::String::SharedPtr msg){
			if(estado_actual_ == Estado::RETRIEVING_ITEM  && msg->data == "Catch"){
				logic();
			}
			else if(estado_actual_ == Estado::DELIVERING_ITEM && msg->data == "Deliver"){
				logic();
			}
			else if(estado_actual_ == Estado::PROCESSING_ORDER &&  msg->data == "Home"){
				logic();
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
				auto id_msg = std_msgs::msg::Int8();
				id_msg.data = pedido[counter_pedido];
				publisher_identification_reference_->publish(id_msg);
				counter_pedido++;
				estado_actual_ = Estado::IDENTIFICATION;
				break;
			}
			case Estado::IDENTIFICATION:{

				if (results == -1){
					estado_actual_ = Estado::PROCESSING_ORDER;
					break;
				}
				RCLCPP_INFO(this->get_logger(), "Objetivo localizado"); 

				auto state_msg = std_msgs::msg::String();
				state_msg.data = "Recogiendo item";
				RCLCPP_INFO(this->get_logger(), "Recogiendo Item"); 
				publisher_state_->publish(state_msg);

				auto coord_msg = geometry_msgs::msg::Point();
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

				auto coord_msg = geometry_msgs::msg::Point();
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

				auto coord_msg = geometry_msgs::msg::Point();
				coord_msg = home_;
				publisher_coordinates_->publish(coord_msg);
				estado_actual_ = Estado::PROCESSING_ORDER;
				break;
			}
			default:
				break;
			}
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
		geometry_msgs::msg::Point destino_;
		geometry_msgs::msg::Point bandeja_;
		geometry_msgs::msg::Point home_;
		
		// Publicadores
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_state_;
		rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_identification_reference_;
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_coordinates_;
		
		// Suscriptores
		rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr subscriber_GUI_;
		rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_ID_;
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_movement_state_;
		
		// Servicios

		
		// Timers
		rclcpp::TimerBase::SharedPtr timer_;
		
};

int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MisionController>());
	rclcpp::shutdown();
	return 0;
}
