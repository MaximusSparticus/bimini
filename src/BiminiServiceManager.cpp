#include "BiminiServiceManager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"
#include "Logger.hpp"

BiminiServiceManager::BiminiServiceManager(
        rclcpp::Node::SharedPtr node,
        std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& buildings)
: m_node(node)
, m_buildings(buildings)
, m_logger(m_node, "ServiceManager")
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
    m_logger.logInfo(std::format("Loading building: {}", request->file_path));
    
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
        
        m_logger.logInfo(std::format("Building loaded with ID: {}", buildingId));
        
    } catch (const std::exception& e) {
        response->success = false;
        response->error_message = std::string("Exception: ") + e.what();
        m_logger.logError("Failed to load building", e);
    }
}

void BiminiServiceManager::handleQueryElement(/* parameters */) {

}

void BiminiServiceManager::handleGenerateNavMap(/* parameters */) {

}

void BiminiServiceManager::handleExportModel(/* parameters */) {

}

std::string BiminiServiceManager::generateUniqueId() const {
    // Use timestamp + random for uniqueness
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 0xFFFF);
    
    std::stringstream ss;
    ss << "building_" << std::hex << timestamp << "_" << std::hex << dis(gen);
    return ss.str();
}
