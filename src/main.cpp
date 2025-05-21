#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @brief BiminiNode is a ROS node which provides an interface to a BIM model to other ROS nodes
 * @todo comments
 * @todo define message interfaces
 */
class BiminiNode : public rclcpp::Node {
 public:
    BiminiNode()
    : Node("bimini")
    , m_timer()
    , m_publisher()
    , m_subscriber()
    , m_count(0)
    , m_lastMsg() {
        // Initialize publisher
        m_publisher = this->create_publisher<std_msgs::msg::String>("output_topic", 10);
    
        // Initialize timer for publishing at regular intervals
        m_timer = this->create_wall_timer(500ms, std::bind(&BiminiNode::timer_callback, this));
    
        // Initialize subscriber
        m_subscriber = this->create_subscription<std_msgs::msg::String>("input_topic", 10,
            std::bind(&BiminiNode::topic_callback, this, std::placeholders::_1));
    
        RCLCPP_INFO(this->get_logger(), "Node initialized");
    }

 private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello from " + std::string(get_name()) + "! Count: " + std::to_string(m_count++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        m_publisher->publish(message);
    }
  
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    
        // Example of processing received data
        m_lastMsg = msg->data;
    }
  
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscriber;
    size_t m_count;
    std::string m_lastMsg;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
  
    // Create and spin the node
    auto node = std::make_shared<BiminiNode>();
  
    // Process callbacks until the user manually kills the process
    rclcpp::spin(node);
  
    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}
