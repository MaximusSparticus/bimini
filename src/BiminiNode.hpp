#ifndef SRC_BIMINI_NODE_HPP_
#define SRC_BIMINI_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_map>

#include "bimini/BIMInterface.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"

// Forward declarations
class BiminiServiceManager;
class BiminiPublisherManager;
class BiminiSubscriberManager;

/**
 * @brief Main BiminiNode class - acts as coordinator between components
 */
class BiminiNode : public rclcpp::Node {
 public:
    // Default constructor
    BiminiNode();

    // Delete copy and move constructors
    BiminiNode(const BiminiNode&) = delete;
    BiminiNode& operator=(const BiminiNode&) = delete;
    BiminiNode(BiminiNode&&) = delete;
    BiminiNode& operator=(BiminiNode&&) = delete;

    // Default destructor defined in source due to forward declarations
    ~BiminiNode();

 private:
    std::unique_ptr<BiminiServiceManager> m_serviceManager;
    std::unique_ptr<BiminiPublisherManager> m_publisherManager;
    std::unique_ptr<BiminiSubscriberManager> m_subscriberManager;

    // Shared state - consider using a state manager for larger systems
    std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>> m_buildings;
};

#endif  // SRC_BIMINI_NODE_HPP_
