#ifndef SRC_LOGGER_HPP_
#define SRC_LOGGER_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

class Logger final {
 public:
    explicit Logger(rclcpp::Node::SharedPtr node, const std::string& logName);

    Logger() = delete;
    Logger(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger& operator=(Logger&&) = delete;

    void logError(const std::string &msg) const;
    void logError(const std::string &msg, const std::exception& e) const;
    void logInfo(const std::string &msg) const;

 private:
    rclcpp::Node::SharedPtr m_node;
    const std::string m_logName;
};

#endif  // SRC_LOGGER_HPP_
