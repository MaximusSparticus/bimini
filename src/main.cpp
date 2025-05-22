#include "rclcpp/rclcpp.hpp"

#include "BiminiNode.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
  
    // Create and spin the node
    auto node = std::make_shared<BiminiNode>();
    node->initialize();
  
    // Process callbacks until the user manually kills the process
    rclcpp::spin(node);
  
    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}
