#ifndef SRC_BIMINI_SUBSCRIBER_MANAGER_HPP_
#define SRC_BIMINI_SUBSCRIBER_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_map>

#include "std_msgs/msg/string.hpp"

#include "bimini/BIMInterface.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"
#include "Logger.hpp"

/**
 * @brief Manages all subscription functionality
 */
class BiminiSubscriberManager {
public:
    explicit BiminiSubscriberManager(rclcpp::Node::SharedPtr node);
    
private:
    void handleInputMessage(const std_msgs::msg::String::SharedPtr msg);

    // Shared Resources
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;

    // Utilities
    Logger m_logger;

    std::string m_lastMessage;
};

#endif  // SRC_BIMINI_SUBSCRIBER_MANAGER_HPP_
