#include "x500_offboard_control/x500_offboard_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
    std::cout << "🚁 Starting x500 Offboard Control Node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<X500OffboardControl>();
        RCLCPP_INFO(node->get_logger(), "🚁 Node created successfully, spinning...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Unknown error occurred" << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    std::cout << "🏁 x500 Offboard Control Node terminated" << std::endl;
    return 0;
}