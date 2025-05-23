#include "BiminiSubscriberManager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"
#include "Logger.hpp"

BiminiSubscriberManager::BiminiSubscriberManager(rclcpp::Node::SharedPtr node)
: m_node(node)
, m_subscription()
, m_logger(m_node, "SubscriberManager")
, m_lastMessage() {
    // Initialize subscriber
    m_subscription = m_node->create_subscription<std_msgs::msg::String>("input_topic", 10,
        std::bind(&BiminiSubscriberManager::handleInputMessage, this, std::placeholders::_1));
}

void BiminiSubscriberManager::handleInputMessage(const std_msgs::msg::String::SharedPtr msg) {
    m_logger.logInfo(std::format("I heard: '{}'", msg->data));
    
    // Example of processing received data
    m_lastMessage = msg->data;
}
