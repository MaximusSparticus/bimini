#include "BiminiServiceManager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"

BiminiServiceManager::BiminiServiceManager(
        rclcpp::Node::SharedPtr node,
        std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& buildings)
: m_node(node)
, m_buildings(buildings)
, m_loadBuildingService() {
    // Create services
    m_loadBuildingService = m_node->create_service<bimini_msgs::srv::LoadBuilding>(
        "load_building",
        std::bind(&BiminiServiceManager::handleLoadBuilding, this,
            std::placeholders::_1, std::placeholders::_2));
}

void BiminiServiceManager::handleLoadBuilding(
        const std::shared_ptr<bimini_msgs::srv::LoadBuilding::Request> request,
        std::shared_ptr<bimini_msgs::srv::LoadBuilding::Response> response) {
    RCLCPP_INFO(m_node->get_logger(), "Loading building: %s", request->file_path.c_str());
    
    try {
        auto bimInterface = std::make_shared<bimini::BIMInterface>();
        
        if (!bimInterface->loadIFC(request->file_path)) {
            response->success = false;
            response->error_message = "Failed to load IFC file";
            return;
        }
        
        std::string buildingId = generateUniqueId();
        m_buildings[buildingId] = bimInterface;
        
        response->success = true;
        response->building_id = buildingId;
        
        RCLCPP_INFO(m_node->get_logger(), "Building loaded with ID: %s", buildingId.c_str());
        
    } catch (const std::exception& e) {
        response->success = false;
        response->error_message = std::string("Exception: ") + e.what();
        RCLCPP_ERROR(m_node->get_logger(), "Failed to load building: %s", e.what());
    }
}

void BiminiServiceManager::handleQueryElement(/* parameters */) {

}

void BiminiServiceManager::handleGenerateNavMap(/* parameters */) {

}

void BiminiServiceManager::handleExportModel(/* parameters */) {

}

std::string BiminiServiceManager::generateUniqueId() const {
    return "todo";
}
