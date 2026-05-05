#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Nodo en C++ para mostrar la interfaz gráfica
class VisualizadorC : public rclcpp::Node {
public:
    VisualizadorC() : Node("visualizador_c") {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/deteccion/output", 10, 
            std::bind(&VisualizadorC::dibujar, this, std::placeholders::_1));
    }

private:
    void dibujar(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convertimos el mensaje de ROS a OpenCV
        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        
        // Añadimos una marca de agua en C++
        cv::putText(cv_ptr->image, "RENDER: C++ ENGINE", cv::Point(20, 40), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        cv::imshow("VISUALIZADOR PROYECTO LAyR", cv_ptr->image);
        cv::waitKey(1);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualizadorC>());
    rclcpp::shutdown();
    return 0;
}

