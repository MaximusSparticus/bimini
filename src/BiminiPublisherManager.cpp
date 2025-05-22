#include "BiminiPublisherManager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"

BiminiPublisherManager::BiminiPublisherManager(
        rclcpp::Node::SharedPtr node,
        const std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& buildings)
: m_node(node)
, m_buildings(buildings)
, m_buildingModelPublisher()
, m_1sTimer()
, m_10sTimer() {
    using namespace std::chrono_literals;
    // Initialize publishers
    m_buildingModelPublisher = m_node->create_publisher<bimini_msgs::msg::BuildingModel>("building_model", 10);
    
    // Initialize timers for the various publishing loop rates
    m_1sTimer = m_node->create_wall_timer(1s, std::bind(&BiminiPublisherManager::loop1Hz, this));
    m_10sTimer = m_node->create_wall_timer(10s, std::bind(&BiminiPublisherManager::loop0p1Hz, this));
}

void BiminiPublisherManager::loop1Hz() {
    // Messages published once a second

    // TODO(zmd): get meaningful data
    auto msg = bimini_msgs::msg::BuildingModel();
    msg.header.stamp = m_node->now();
    msg.header.frame_id = "world";
    msg.building_id = "building_001";
    msg.name = "Office Building";
    msg.source_file = "some_path.ifc";
    RCLCPP_INFO(m_node->get_logger(), std::format(
        "msg.header.stamp = (), msg.header.frame_id = {}, msg.building_id = {}, msg.name = {}, msg.source_file = {}",
        //msg.header.stamp,
        msg.header.frame_id,
        msg.building_id,
        msg.name,
        msg.source_file).c_str());
    m_buildingModelPublisher->publish(msg);
}

void BiminiPublisherManager::loop0p1Hz() {
    // TODO(zmd): messages published once every 10 seconds
}
