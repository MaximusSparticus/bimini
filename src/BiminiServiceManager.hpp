#ifndef SRC_BIMINI_SERVICE_MANAGER_HPP_
#define SRC_BIMINI_SERVICE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_map>

#include "bimini/BIMInterface.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"
#include "Logger.hpp"

/**
 * @brief Manages all service-related functionality
 */
class BiminiServiceManager {
 public:
    explicit BiminiServiceManager(
        rclcpp::Node::SharedPtr node,
        std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& buildings);

 private:
    void handleLoadBuilding(
        const std::shared_ptr<bimini_msgs::srv::LoadBuilding::Request> request,
        std::shared_ptr<bimini_msgs::srv::LoadBuilding::Response> response);

    void handleQueryElement(/* parameters */);
    void handleGenerateNavMap(/* parameters */);
    void handleExportModel(/* parameters */);

    std::string generateUniqueId() const;

    // Shared Resources
    rclcpp::Node::SharedPtr m_node;
    std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& m_buildings;

    // Utilities
    Logger m_logger;

    // Services
    rclcpp::Service<bimini_msgs::srv::LoadBuilding>::SharedPtr m_loadBuildingService;
    // Add other services as needed
};

#endif  // SRC_BIMINI_SERVICE_MANAGER_HPP_
