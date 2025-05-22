#ifndef SRC_BIMINI_PUBLISHER_MANAGER_HPP_
#define SRC_BIMINI_PUBLISHER_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_map>

#include "bimini/BIMInterface.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"
#include "Logger.hpp"


/**
 * @brief Manages all publishing functionality with different strategies
 */
class BiminiPublisherManager {
 public:
    explicit BiminiPublisherManager(
        rclcpp::Node::SharedPtr node,
        const std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& buildings);

    void startPublishing(std::chrono::milliseconds interval);
    void stopPublishing();

 private:
    void publish();

    // Shared Resources
    rclcpp::Node::SharedPtr m_node;
    const std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& m_buildings;

    // Utilities
    Logger m_logger;

    // Publishers
    rclcpp::Publisher<bimini_msgs::msg::BuildingModel>::SharedPtr m_buildingModelPublisher;

    // Timers
    rclcpp::TimerBase::SharedPtr m_timer;
};

#endif  // SRC_BIMINI_PUBLISHER_MANAGER_HPP_
