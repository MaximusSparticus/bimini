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
, m_logger(m_node, "PublisherManager")
, m_buildingModelPublisher()
, m_timer() {
    // Initialize publishers
    m_buildingModelPublisher = m_node->create_publisher<bimini_msgs::msg::BuildingModel>("building_model", 10);
}

void BiminiPublisherManager::startPublishing(std::chrono::milliseconds interval) {
    if (m_timer) {
        m_timer->cancel();
    }

    m_timer = m_node->create_wall_timer(interval, std::bind(&BiminiPublisherManager::publish, this));

    m_logger.logInfo(std::format("Started publishing every {} [ms]", interval.count()));
}

void BiminiPublisherManager::stopPublishing() {
    if (m_timer) {
        m_timer->cancel();
        m_timer.reset();
        m_logger.logInfo("Stopped publishing");
    }
}

void BiminiPublisherManager::publish() {
    // Messages published once a second

    // TODO(zmd): get meaningful data
    auto msg = bimini_msgs::msg::BuildingModel();
    msg.header.stamp = m_node->now();
    msg.header.frame_id = "world";
    msg.building_id = "building_001";
    msg.name = "Office Building";
    msg.source_file = "some_path.ifc";
    m_logger.logInfo(std::format(
        "msg.header.stamp = (), msg.header.frame_id = {}, msg.building_id = {}, msg.name = {}, msg.source_file = {}",
        //msg.header.stamp,
        msg.header.frame_id,
        msg.building_id,
        msg.name,
        msg.source_file));
    m_buildingModelPublisher->publish(msg);
}
