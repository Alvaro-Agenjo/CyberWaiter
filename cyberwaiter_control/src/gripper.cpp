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
        io_client_ = this->create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");

        service_G_ = this->create_service<cyberwaiter_msgs::srv::Gripper>
        ("close_gripper", std::bind(&GripperActuator::set_gripper, this, std::placeholders::_1, std::placeholders::_2));



    }

private:
    bool set_io(int pin, int state){
        if (!io_client_->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "El servicio setIO no está disponible.");
            return false;
        }
        auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
        request->fun = 1; // Digital Output
        request->pin = pin;
        request->state = state;
        RCLCPP_INFO(this->get_logger(), "send...");
        auto future = io_client_->async_send_request(request);
        
        auto response = future.get();
        return response->success;
    }
    
    bool open(){
        int counter = 0;
        bool flag = true;
        RCLCPP_INFO(this->get_logger(), "Abriendo gripper...");
        while (counter <3){
            if (!set_io(17, 0) && !flag){
                RCLCPP_ERROR(this->get_logger(), "Not comunicate gripper phase I, attempt %d.", counter);
                counter ++;
                continue;
            }
            flag = false;
            if (!set_io(16, 1)){
                RCLCPP_ERROR(this->get_logger(), "Not comunicate gripper phase II, attempt %d.", counter);
                counter ++;
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Gripper abierto.");
            return true;
        }
        return false;
        
    }
    bool close(){
        int counter = 0;
        bool flag = true;
        RCLCPP_INFO(this->get_logger(), "Cerrando gripper...");
        while (counter <3){
            if (!set_io(16, 0) && !flag){
                RCLCPP_ERROR(this->get_logger(), "Not comunicate gripper phase I, attempt %d.", counter);
                counter ++;
                continue;
            }
            flag = false;
            if (!set_io(17, 1)){
                RCLCPP_ERROR(this->get_logger(), "Not comunicate gripper phase II, attempt %d.", counter);
                counter ++;
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Gripper cerrado.");
            return true;
        }
        return false;
        
    }
    void set_gripper(const std::shared_ptr<cyberwaiter_msgs::srv::Gripper::Request> request,
                     std::shared_ptr<cyberwaiter_msgs::srv::Gripper::Response> response){
        bool result = false;
        if (request->state)
            result = close();
        else
            result = open();

        response->success = result;
    }
    rclcpp::Service<cyberwaiter_msgs::srv::Gripper>::SharedPtr service_G_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr io_client_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperActuator>());
    rclcpp::shutdown();
    return 0;
}
