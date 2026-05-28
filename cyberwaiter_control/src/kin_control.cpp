#include <memory>
#include <string>
#include <vector>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "urdf/model.h"

#define deg2rad(x) ((x)*M_PI/180.0)
#define vel 0.3
#define TCP_offset 0.145//0.145
using namespace std::chrono_literals;

class Kinematic_controler : public rclcpp::Node {
public:
    
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    
    Kinematic_controler() : Node("kinematic_controler") {

        rclcpp::QoS best_effort__Volatile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        rclcpp::QoS reliable__Volatile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        rclcpp::QoS transient_local = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>("/robot_description", transient_local,
            std::bind(&Kinematic_controler::robot_description_callback, this, std::placeholders::_1));

        cartesian_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("/goals/goal_coord", reliable__Volatile, 
            std::bind(&Kinematic_controler::cartesian_callback, this, std::placeholders::_1));

        movement_state = this->create_publisher<std_msgs::msg::String>("/goals/state", reliable__Volatile);

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", best_effort__Volatile,
             std::bind(&Kinematic_controler::joint_state_callback, this, std::placeholders::_1));


        RCLCPP_INFO(this->get_logger(), "Esperando el URDF desde /robot_description...");

        /*Accion */
        
        this->client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, //nodo
            "/scaled_joint_trajectory_controller/follow_joint_trajectory" //nombre de la accion
        );       

        }

private:
    // CALLBACK PARA EL URDF
    void robot_description_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (kdl_initialized_) return; // Ya lo tenemos

        urdf::Model urdf_model;
        if (!urdf_model.initString(msg->data)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF for limits");
            return;
        }

        KDL::Tree tree;
        if (!kdl_parser::treeFromString(msg->data, tree)) {
            RCLCPP_ERROR(this->get_logger(), "Fallo al parsear el URDF recibido.");
            return;
        }

        // Configurar la cadena y solvers (igual que antes)
        if (!tree.getChain("base", "tool0", chain_)) {
            RCLCPP_ERROR(this->get_logger(), "No se encontró la cadena entre base_link y tool0");
            return;
        }

        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        vik_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);
        
        // Límites genéricos (idealmente leerlos del URDF dinámicamente)
        current_joint_state_.resize(chain_.getNrOfJoints());

        

        KDL::JntArray q_min(chain_.getNrOfJoints()), q_max(chain_.getNrOfJoints());
        unsigned int joint_counter = 0;
        for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
            KDL::Joint joint = chain_.getSegment(i).getJoint();

            if (joint.getType() == KDL::Joint::None) continue;

            auto urdf_joint = urdf_model.getJoint(joint.getName());

            if (urdf_joint && urdf_joint->limits && (joint.getName() != "wrist_3_joint")){// && joint.getName() != "shoulder_lift_joint")) {
                q_min(joint_counter) = urdf_joint->limits->lower;
                q_max(joint_counter) = urdf_joint->limits->upper;
            }
            // else if (joint.getName() == "shoulder_lift_joint") {
            //     q_min(joint_counter) = 0; // 0 grados
            //     q_max(joint_counter) = -3.14159;  // 90 grados
            // }
            else {
                q_min(joint_counter) = -3.14159*2;
                q_max(joint_counter) = 3.14159*2;
            }

            RCLCPP_INFO(this->get_logger(), "Joint %d (Index %d): %s, Limits: [%.2f, %.2f]", 
                        joint_counter, i, joint.getName().c_str(), 
                        q_min(joint_counter), q_max(joint_counter));
            
            joint_counter++;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
            chain_, q_min, q_max, *fk_solver_, *vik_solver_, 5000, 1e-3);

        kdl_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "KDL inicializado correctamente con el URDF del sistema.");
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!kdl_initialized_) return; // Asegurarnos de que KDL esté listo antes de procesar estados
        


        for (size_t i = 0; i < msg->name.size(); ++i) {
            // En C++, el operador == está sobrecargado para comparar el contenido de std::string
            if (msg->name[i] == "shoulder_pan_joint") {
                current_joint_state_(0) = msg->position[i];
            } 
            else if (msg->name[i] == "shoulder_lift_joint") {
                current_joint_state_(1) = msg->position[i];
            } 
            else if (msg->name[i] == "elbow_joint") {
                current_joint_state_(2) = msg->position[i];
            } 
            else if (msg->name[i] == "wrist_1_joint") {
                current_joint_state_(3) = msg->position[i];
            } 
            else if (msg->name[i] == "wrist_2_joint") {
                current_joint_state_(4) = msg->position[i];
            } 
            else if (msg->name[i] == "wrist_3_joint") {
                current_joint_state_(5) = msg->position[i];
            }
        }

         //RCLCPP_INFO(this->get_logger(), "Coordenadas articulares cb: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", current_joint_state_(0), current_joint_state_(1), current_joint_state_(2), current_joint_state_(3), current_joint_state_(4), current_joint_state_(5));
    }

    void cartesian_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
        goal.M = KDL::Rotation::Quaternion(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
        goal.p = KDL::Vector(
        msg->position.x,
        msg->position.y,
        msg->position.z);

    
        KDL::Frame tool0_deseado = goal * KDL::Frame(KDL::Vector(0.0, 0.0, -TCP_offset)); //

        // // 3. Pasamos al solver la pose de tool0_deseado (que mantendrá limpia la cinemática nativa)
        KDL::JntArray target_joints(chain_.getNrOfJoints());
        int ret = ik_solver_->CartToJnt(current_joint_state_, tool0_deseado, target_joints);

        // KDL::Frame current_fk_pose;
        // fk_solver_->JntToCart(current_joint_state_, current_fk_pose);

        // RCLCPP_INFO(this->get_logger(), "Pose actual FK: X:%.3f Y:%.3f Z:%.3f", 
        //             current_fk_pose.p.x(), current_fk_pose.p.y(), current_fk_pose.p.z());
        RCLCPP_INFO(this->get_logger(), "[Kin Solver] Objetivo recibido desde 'NAVIGATOR': X:%.3f Y:%.3f Z:%.3f", 
                    goal.p.x(), goal.p.y(), goal.p.z());
        RCLCPP_INFO(this->get_logger(), "[Kin Solver] Pose de la muñeca tras compensar TCP: X:%.3f Y:%.3f Z:%.3f", 
                    tool0_deseado.p.x(), tool0_deseado.p.y(), tool0_deseado.p.z());
        
        if (ret >= 0) {
            RCLCPP_INFO(this->get_logger(), "[Kin Solver] Solución encontrada, enviando orden...");
            std::vector<double> target_positions(target_joints.rows());
            for (size_t i = 0; i < target_joints.rows(); ++i) {
                target_positions[i] = target_joints(i);
            }

            double distance = sqrt(pow(goal.p.x(), 2) + pow(goal.p.y(), 2) + pow(goal.p.z(), 2));
            send_goal(target_positions, distance);

        }
        else{
            RCLCPP_WARN(this->get_logger(), "[Kin Solver] [ERROR IK] No se pudo resolver la cinemática inversa para el objetivo recibido. Posicion de escape...");
            // std::vector<double> home_positions = {deg2rad(-24.2), deg2rad(-56.03), deg2rad(80.68), deg2rad(-27.18), deg2rad(-47.22), deg2rad(161.06)};
            std::vector<double> home_positions = {deg2rad(4.02), deg2rad(-60.84), deg2rad(103.68), deg2rad(-47.9), deg2rad(-31.91), deg2rad(165.07)};
            send_goal(home_positions, 3);

            auto msg = std_msgs::msg::String();
            msg.data = "RETRY";
            movement_state->publish(msg);
        }
  }
    
    void send_goal(std::vector<double> target_positions, double distance){
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Servidor de acción no disponible después de esperar");
        return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();

        // Configuración de los nombres de los joints
        goal_msg.trajectory.joint_names = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        };

        // Definición del punto de trayectoria
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = target_positions;
        point.time_from_start = rclcpp::Duration(std::chrono::milliseconds(static_cast<int>((distance / vel + 0.8) * 1000)));

        goal_msg.trajectory.points.push_back(point);

        RCLCPP_INFO(this->get_logger(), "Enviando objetivo de trayectoria...");

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
        std::bind(&Kinematic_controler::goal_response_callback, this, std::placeholders::_1);
        
        send_goal_options.result_callback =
        std::bind(&Kinematic_controler::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:{
                RCLCPP_INFO(this->get_logger(), "¡Movimiento completado con éxito!");
                
                auto msg = std_msgs::msg::String();
                msg.data = "OK";
                movement_state->publish(msg);              
                break;
            }
            case rclcpp_action::ResultCode::ABORTED:{
                RCLCPP_ERROR(this->get_logger(), "El movimiento fue abortado");
            
                auto msg = std_msgs::msg::String();
                msg.data = "NOK";
                movement_state->publish(msg);
                break;
            }
            case rclcpp_action::ResultCode::CANCELED:{
                RCLCPP_ERROR(this->get_logger(), "El movimiento fue cancelado");
            
                auto msg = std_msgs::msg::String();
                msg.data = "NOK";
                movement_state->publish(msg);           
                break;
            }
        default:
            RCLCPP_ERROR(this->get_logger(), "Código de resultado desconocido");
            break;
        }
    }
    
    
    bool kdl_initialized_ = false;
    bool first_run_ = false;

    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> vik_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;

    KDL::JntArray current_joint_state_;
    KDL::Frame goal;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr movement_state;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_text_sub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr cartesian_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Kinematic_controler>());
    rclcpp::shutdown();
    return 0;
}
