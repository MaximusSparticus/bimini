#ifndef SRC_BIMINI_PUBLISHER_MANAGER_HPP_
#define SRC_BIMINI_PUBLISHER_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_map>

#include "bimini/BIMInterface.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"


/**
 * @brief Manages all publishing functionality with different strategies
 */
class BiminiPublisherManager {
 public:
    explicit BiminiPublisherManager(
        rclcpp::Node::SharedPtr node,
        const std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& buildings);

 private:
    void loop1Hz();
    void loop0p1Hz();

    // Shared Resources
    rclcpp::Node::SharedPtr m_node;
    const std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& m_buildings;

    // Publishers
    rclcpp::Publisher<bimini_msgs::msg::BuildingModel>::SharedPtr m_buildingModelPublisher;

    // Timers
    rclcpp::TimerBase::SharedPtr m_1sTimer;
    rclcpp::TimerBase::SharedPtr m_10sTimer;
};

#endif  // SRC_BIMINI_PUBLISHER_MANAGER_HPP_
