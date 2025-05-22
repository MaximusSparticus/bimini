#include "BiminiNode.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"
#include "BiminiServiceManager.hpp"
#include "BiminiPublisherManager.hpp"
#include "BiminiSubscriberManager.hpp"


BiminiNode::BiminiNode()
: Node("bimini")
, m_serviceManager()
, m_publisherManager()
, m_subscriberManager()
, m_buildings() {
}

BiminiNode::~BiminiNode() = default;

void BiminiNode::initialize(const BiminiNodeConfig& config) {
    // Create managers
    m_serviceManager = std::make_unique<BiminiServiceManager>(shared_from_this(), m_buildings);
    m_publisherManager = std::make_unique<BiminiPublisherManager>(shared_from_this(), m_buildings);
    m_subscriberManager = std::make_unique<BiminiSubscriberManager>(shared_from_this());

    // Configure the node
    if (config.enablePeriodicPublishing) {
        m_publisherManager->startPublishing(config.publishInterval);
    } else {
        m_publisherManager->stopPublishing();
    }

    RCLCPP_INFO(this->get_logger(), "BiminiNode initialized");
}
