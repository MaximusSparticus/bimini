#include "Logger.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>

Logger::Logger(rclcpp::Node::SharedPtr node, const std::string& logName)
: m_node(node)
, m_logName(logName)
{}

void Logger::logError(const std::string &msg) const {
    RCLCPP_ERROR(m_node->get_logger(), std::format("[{}]: {}", m_logName, msg).c_str());
}

void Logger::logError(const std::string &msg, const std::exception& e) const {
    RCLCPP_ERROR(m_node->get_logger(), std::format(
        "[{}]: {} with exception: '{}'", m_logName, msg, e.what()).c_str());
}

void Logger::logInfo(const std::string &msg) const {
    RCLCPP_INFO(m_node->get_logger(), std::format("[{}]: {}", m_logName, msg).c_str());
}
