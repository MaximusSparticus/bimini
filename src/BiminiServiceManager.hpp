#ifndef SRC_BIMINI_SERVICE_MANAGER_HPP_
#define SRC_BIMINI_SERVICE_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <unordered_map>

#include "bimini/BIMInterface.hpp"
#include "bimini_msgs/srv/load_building.hpp"
#include "bimini_msgs/srv/query_building_element.hpp"
#include "bimini_msgs/srv/generate_navigation_map.hpp"
#include "bimini_msgs/srv/export_simulation_model.hpp"
#include "Logger.hpp"

/**
 * @brief Manages all service-related functionality for BIM operations
 */
class BiminiServiceManager {
public:
    explicit BiminiServiceManager(
        rclcpp::Node::SharedPtr node,
        std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& buildings);

    ~BiminiServiceManager() = default;

private:
    // Service callback handlers
    void handleLoadBuilding(
        const std::shared_ptr<bimini_msgs::srv::LoadBuilding::Request> request,
        std::shared_ptr<bimini_msgs::srv::LoadBuilding::Response> response);

    void handleQueryBuildingElement(
        const std::shared_ptr<bimini_msgs::srv::QueryBuildingElement::Request> request,
        std::shared_ptr<bimini_msgs::srv::QueryBuildingElement::Response> response);

    void handleGenerateNavigationMap(
        const std::shared_ptr<bimini_msgs::srv::GenerateNavigationMap::Request> request,
        std::shared_ptr<bimini_msgs::srv::GenerateNavigationMap::Response> response);

    void handleExportSimulationModel(
        const std::shared_ptr<bimini_msgs::srv::ExportSimulationModel::Request> request,
        std::shared_ptr<bimini_msgs::srv::ExportSimulationModel::Response> response);

    // Helper methods
    std::string generateUniqueId() const;
    std::string generateMapId() const;
    bool validateBuildingExists(const std::string& buildingId) const;
    
    // Navigation map generation helpers
    bool generateFloorMap(const std::string& buildingId, 
                         const std::string& floorId,
                         double resolution, 
                         double agentRadius,
                         nav_msgs::msg::OccupancyGrid& gridMap);
    
    // Export helpers
    bool exportToURDF(const std::string& buildingId, 
                     const std::string& destinationPath,
                     bool includeSemantics,
                     bool includeCollision,
                     bool simplifyGeometry,
                     std::vector<std::string>& exportedFiles);
    
    bool exportToSDF(const std::string& buildingId, 
                    const std::string& destinationPath,
                    bool includeSemantics,
                    bool includeCollision,
                    bool simplifyGeometry,
                    std::vector<std::string>& exportedFiles);

    // Shared Resources
    rclcpp::Node::SharedPtr m_node;
    std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& m_buildings;

    // Utilities
    Logger m_logger;

    // Services
    rclcpp::Service<bimini_msgs::srv::LoadBuilding>::SharedPtr m_loadBuildingService;
    rclcpp::Service<bimini_msgs::srv::QueryBuildingElement>::SharedPtr m_queryElementService;
    rclcpp::Service<bimini_msgs::srv::GenerateNavigationMap>::SharedPtr m_generateNavMapService;
    rclcpp::Service<bimini_msgs::srv::ExportSimulationModel>::SharedPtr m_exportModelService;
};

#endif  // SRC_BIMINI_SERVICE_MANAGER_HPP_
