#include "BiminiServiceManager.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

BiminiServiceManager::BiminiServiceManager(
        rclcpp::Node::SharedPtr node,
        std::unordered_map<std::string, std::shared_ptr<bimini::BIMInterface>>& buildings)
: m_node(node)
, m_buildings(buildings)
, m_logger(m_node, "ServiceManager") {
    // Create services
    m_loadBuildingService = m_node->create_service<bimini_msgs::srv::LoadBuilding>(
        "load_building",
        std::bind(&BiminiServiceManager::handleLoadBuilding, this,
            std::placeholders::_1, std::placeholders::_2));

    m_queryElementService = m_node->create_service<bimini_msgs::srv::QueryBuildingElement>(
        "query_building_element",
        std::bind(&BiminiServiceManager::handleQueryBuildingElement, this,
            std::placeholders::_1, std::placeholders::_2));

    m_generateNavMapService = m_node->create_service<bimini_msgs::srv::GenerateNavigationMap>(
        "generate_navigation_map",
        std::bind(&BiminiServiceManager::handleGenerateNavigationMap, this,
            std::placeholders::_1, std::placeholders::_2));

    m_exportModelService = m_node->create_service<bimini_msgs::srv::ExportSimulationModel>(
        "export_simulation_model",
        std::bind(&BiminiServiceManager::handleExportSimulationModel, this,
            std::placeholders::_1, std::placeholders::_2));

    m_logger.logInfo("All BIM services initialized successfully");
}

void BiminiServiceManager::handleLoadBuilding(
        const std::shared_ptr<bimini_msgs::srv::LoadBuilding::Request> request,
        std::shared_ptr<bimini_msgs::srv::LoadBuilding::Response> response) {
    m_logger.logInfo(std::format("Loading building from: {}", request->file_path));
    
    try {
        // Validate file exists
        if (!std::filesystem::exists(request->file_path)) {
            response->success = false;
            response->error_message = "IFC file not found: " + request->file_path;
            m_logger.logError(std::format("File not found: {}", request->file_path));
            return;
        }

        // Create BIM interface
        auto bimInterface = std::make_shared<bimini::BIMInterface>();
        
        // Load IFC file
        if (!bimInterface->loadIFC(request->file_path)) {
            response->success = false;
            response->error_message = "Failed to parse IFC file: " + request->file_path;
            m_logger.logError(std::format("Failed to parse IFC: {}", request->file_path));
            return;
        }
        
        // Optional validation
        if (request->validate_file) {
            // TODO: Add IFC validation logic here
            m_logger.logInfo("IFC validation requested - implementing validation...");
        }
        
        // Generate unique ID and store building
        std::string buildingId = generateUniqueId();
        m_buildings[buildingId] = bimInterface;
        
        response->success = true;
        response->building_id = buildingId;
        
        m_logger.logInfo(std::format("Building loaded successfully with ID: {}", buildingId));
        
    } catch (const std::exception& e) {
        response->success = false;
        response->error_message = std::string("Exception during loading: ") + e.what();
        m_logger.logError("Failed to load building", e);
    }
}

void BiminiServiceManager::handleQueryBuildingElement(
        const std::shared_ptr<bimini_msgs::srv::QueryBuildingElement::Request> request,
        std::shared_ptr<bimini_msgs::srv::QueryBuildingElement::Response> response) {
    m_logger.logInfo(std::format("Querying building {} for {} elements", 
                                request->building_id, request->query_type));
    
    try {
        // Validate building exists
        if (!validateBuildingExists(request->building_id)) {
            response->found = false;
            response->error_message = "Building not found: " + request->building_id;
            return;
        }

        auto building = m_buildings[request->building_id];
        
        // Handle different query types
        if (request->query_type == "ELEMENT" && !request->element_id.empty()) {
            // Query specific element by ID
            bimini_msgs::msg::BuildingElement element;
            element.id = request->element_id;
            element.type = "WALL"; // TODO: Get actual type from BIM data
            
            // TODO: Extract actual element data from BIMInterface
            // This is a placeholder implementation
            element.pose.position.x = 0.0;
            element.pose.position.y = 0.0;
            element.pose.position.z = 0.0;
            element.dimensions.x = 1.0;
            element.dimensions.y = 0.2;
            element.dimensions.z = 3.0;
            element.material = "concrete";
            element.is_structural = true;
            
            response->elements.push_back(element);
            response->found = true;
            
        } else if (request->query_type == "ROOM") {
            // Query all rooms
            // TODO: Implement room querying from BIMInterface
            m_logger.logInfo("Room querying not yet implemented");
            response->found = false;
            response->error_message = "Room querying not yet implemented";
            
        } else if (request->query_type == "FLOOR") {
            // Query all floors
            // TODO: Implement floor querying from BIMInterface
            m_logger.logInfo("Floor querying not yet implemented");
            response->found = false;
            response->error_message = "Floor querying not yet implemented";
            
        } else {
            // Spatial query by position and radius
            if (request->radius > 0.0) {
                // TODO: Implement spatial querying
                m_logger.logInfo(std::format("Spatial query at ({}, {}, {}) with radius {}",
                                           request->position.x, request->position.y, 
                                           request->position.z, request->radius));
                response->found = false;
                response->error_message = "Spatial querying not yet implemented";
            } else {
                response->found = false;
                response->error_message = "Invalid query parameters";
            }
        }
        
    } catch (const std::exception& e) {
        response->found = false;
        response->error_message = std::string("Exception during query: ") + e.what();
        m_logger.logError("Failed to query building element", e);
    }
}

void BiminiServiceManager::handleGenerateNavigationMap(
        const std::shared_ptr<bimini_msgs::srv::GenerateNavigationMap::Request> request,
        std::shared_ptr<bimini_msgs::srv::GenerateNavigationMap::Response> response) {
    m_logger.logInfo(std::format("Generating navigation map for building: {}", request->building_id));
    
    try {
        // Validate building exists
        if (!validateBuildingExists(request->building_id)) {
            response->success = false;
            response->error_message = "Building not found: " + request->building_id;
            return;
        }

        auto building = m_buildings[request->building_id];
        
        // Generate unique map ID
        std::string mapId = generateMapId();
        
        // Process floor selection
        std::vector<std::string> floorsToProcess;
        if (request->included_floors.empty()) {
            // TODO: Get all floor IDs from building
            floorsToProcess.push_back("ground_floor"); // Placeholder
        } else {
            floorsToProcess = request->included_floors;
        }
        
        // Generate maps for each floor
        std::vector<nav_msgs::msg::OccupancyGrid> floorMaps;
        for (const auto& floorId : floorsToProcess) {
            nav_msgs::msg::OccupancyGrid gridMap;
            
            if (generateFloorMap(request->building_id, floorId, 
                               request->resolution, request->agent_radius, gridMap)) {
                floorMaps.push_back(gridMap);
                m_logger.logInfo(std::format("Generated map for floor: {}", floorId));
            } else {
                m_logger.logError(std::format("Failed to generate map for floor: {}", floorId));
            }
        }
        
        // TODO: Generate 3D navigation mesh if needed
        std::string meshResource = "";
        
        response->success = !floorMaps.empty();
        response->map_id = mapId;
        response->floor_maps = floorMaps;
        response->mesh_resource = meshResource;
        
        if (response->success) {
            m_logger.logInfo(std::format("Navigation map generated with ID: {}", mapId));
        } else {
            response->error_message = "Failed to generate any floor maps";
        }
        
    } catch (const std::exception& e) {
        response->success = false;
        response->error_message = std::string("Exception during map generation: ") + e.what();
        m_logger.logError("Failed to generate navigation map", e);
    }
}

void BiminiServiceManager::handleExportSimulationModel(
        const std::shared_ptr<bimini_msgs::srv::ExportSimulationModel::Request> request,
        std::shared_ptr<bimini_msgs::srv::ExportSimulationModel::Response> response) {
    m_logger.logInfo(std::format("Exporting building {} to {} format at: {}", 
                                request->building_id, request->format, request->destination_path));
    
    try {
        // Validate building exists
        if (!validateBuildingExists(request->building_id)) {
            response->success = false;
            response->error_message = "Building not found: " + request->building_id;
            return;
        }

        // Create destination directory if it doesn't exist
        std::filesystem::path destPath(request->destination_path);
        if (!std::filesystem::exists(destPath)) {
            std::filesystem::create_directories(destPath);
        }

        std::vector<std::string> exportedFiles;
        std::string mainFile;
        bool exportSuccess = false;
        
        // Handle different export formats
        if (request->format == "URDF") {
            exportSuccess = exportToURDF(request->building_id, request->destination_path,
                                       request->include_semantics, request->include_collision,
                                       request->simplify_geometry, exportedFiles);
            if (exportSuccess && !exportedFiles.empty()) {
                mainFile = exportedFiles[0]; // First file is typically the main URDF
            }
            
        } else if (request->format == "SDF") {
            exportSuccess = exportToSDF(request->building_id, request->destination_path,
                                      request->include_semantics, request->include_collision,
                                      request->simplify_geometry, exportedFiles);
            if (exportSuccess && !exportedFiles.empty()) {
                mainFile = exportedFiles[0]; // First file is typically the main SDF
            }
            
        } else if (request->format == "GAZEBO") {
            // Gazebo typically uses SDF format
            exportSuccess = exportToSDF(request->building_id, request->destination_path,
                                      request->include_semantics, request->include_collision,
                                      request->simplify_geometry, exportedFiles);
            if (exportSuccess && !exportedFiles.empty()) {
                mainFile = exportedFiles[0];
            }
            
        } else {
            response->success = false;
            response->error_message = "Unsupported export format: " + request->format;
            return;
        }
        
        response->success = exportSuccess;
        response->exported_files = exportedFiles;
        response->main_file = mainFile;
        
        if (exportSuccess) {
            m_logger.logInfo(std::format("Export completed. {} files exported to: {}", 
                                       exportedFiles.size(), request->destination_path));
        } else {
            response->error_message = "Export failed for format: " + request->format;
        }
        
    } catch (const std::exception& e) {
        response->success = false;
        response->error_message = std::string("Exception during export: ") + e.what();
        m_logger.logError("Failed to export simulation model", e);
    }
}

// Helper method implementations
std::string BiminiServiceManager::generateUniqueId() const {
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

std::string BiminiServiceManager::generateMapId() const {
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 0xFFFF);
    
    std::stringstream ss;
    ss << "map_" << std::hex << timestamp << "_" << std::hex << dis(gen);
    return ss.str();
}

bool BiminiServiceManager::validateBuildingExists(const std::string& buildingId) const {
    bool exists = m_buildings.find(buildingId) != m_buildings.end();
    if (!exists) {
        m_logger.logError(std::format("Building ID not found: {}", buildingId));
    }
    return exists;
}

bool BiminiServiceManager::generateFloorMap(const std::string& buildingId, 
                                          const std::string& floorId,
                                          double resolution, 
                                          double agentRadius,
                                          nav_msgs::msg::OccupancyGrid& gridMap) {
    // TODO: Implement actual floor map generation from BIM data
    // This is a placeholder implementation
    
    m_logger.logInfo(std::format("Generating floor map for {} with resolution {} and agent radius {}", 
                                floorId, resolution, agentRadius));
    
    // Create a simple placeholder grid
    gridMap.header.stamp = m_node->now();
    gridMap.header.frame_id = "map";
    gridMap.info.resolution = static_cast<float>(resolution);
    gridMap.info.width = 100; // 10m x 10m at 0.1m resolution
    gridMap.info.height = 100;
    gridMap.info.origin.position.x = -5.0;
    gridMap.info.origin.position.y = -5.0;
    gridMap.info.origin.position.z = 0.0;
    
    // Initialize with free space (0 = free, 100 = occupied, -1 = unknown)
    gridMap.data.resize(gridMap.info.width * gridMap.info.height, 0);
    
    // Add some placeholder walls
    for (int i = 0; i < 100; ++i) {
        gridMap.data[i] = 100; // Top wall
        gridMap.data[99 * 100 + i] = 100; // Bottom wall
        gridMap.data[i * 100] = 100; // Left wall
        gridMap.data[i * 100 + 99] = 100; // Right wall
    }
    
    return true;
}

bool BiminiServiceManager::exportToURDF(const std::string& building_id, 
                                       const std::string& destinationPath,
                                       bool includeSemantics,
                                       bool includeCollision,
                                       bool simplifyGeometry,
                                       std::vector<std::string>& exportedFiles) {
    // TODO: Implement actual URDF export from BIM data
    // This is a placeholder implementation
    
    std::filesystem::path urdfPath = std::filesystem::path(destinationPath) / (building_id + ".urdf");
    
    std::ofstream urdfFile(urdfPath);
    if (!urdfFile.is_open()) {
        m_logger.logError(std::format("Failed to create URDF file: {}", urdfPath.string()));
        return false;
    }
    
    // Write basic URDF structure
    urdfFile << "<?xml version=\"1.0\"?>\n";
    urdfFile << "<robot name=\"" << building_id << "\">\n";
    urdfFile << "  <!-- Building exported from BIM data -->\n";
    urdfFile << "  <link name=\"base_link\">\n";
    
    if (includeCollision) {
        urdfFile << "    <collision>\n";
        urdfFile << "      <geometry>\n";
        urdfFile << "        <box size=\"10 10 3\"/>\n";
        urdfFile << "      </geometry>\n";
        urdfFile << "    </collision>\n";
    }
    
    urdfFile << "    <visual>\n";
    urdfFile << "      <geometry>\n";
    urdfFile << "        <box size=\"10 10 3\"/>\n";
    urdfFile << "      </geometry>\n";
    urdfFile << "    </visual>\n";
    urdfFile << "  </link>\n";
    urdfFile << "</robot>\n";
    
    urdfFile.close();
    
    exportedFiles.push_back(urdfPath.string());
    m_logger.logInfo(std::format("URDF exported to: {}", urdfPath.string()));
    
    return true;
}

bool BiminiServiceManager::exportToSDF(const std::string& building_id, 
                                      const std::string& destinationPath,
                                      bool includeSemantics,
                                      bool includeCollision,
                                      bool simplifyGeometry,
                                      std::vector<std::string>& exportedFiles) {
    // TODO: Implement actual SDF export from BIM data
    // This is a placeholder implementation
    
    std::filesystem::path sdfPath = std::filesystem::path(destinationPath) / (building_id + ".sdf");
    
    std::ofstream sdfFile(sdfPath);
    if (!sdfFile.is_open()) {
        m_logger.logError(std::format("Failed to create SDF file: {}", sdfPath.string()));
        return false;
    }
    
    // Write basic SDF structure
    sdfFile << "<?xml version=\"1.0\"?>\n";
    sdfFile << "<sdf version=\"1.7\">\n";
    sdfFile << "  <model name=\"" << building_id << "\">\n";
    sdfFile << "    <!-- Building exported from BIM data -->\n";
    sdfFile << "    <link name=\"base_link\">\n";
    
    if (includeCollision) {
        sdfFile << "      <collision name=\"collision\">\n";
        sdfFile << "        <geometry>\n";
        sdfFile << "          <box><size>10 10 3</size></box>\n";
        sdfFile << "        </geometry>\n";
        sdfFile << "      </collision>\n";
    }
    
    sdfFile << "      <visual name=\"visual\">\n";
    sdfFile << "        <geometry>\n";
    sdfFile << "          <box><size>10 10 3</size></box>\n";
    sdfFile << "        </geometry>\n";
    sdfFile << "      </visual>\n";
    sdfFile << "    </link>\n";
    sdfFile << "  </model>\n";
    sdfFile << "</sdf>\n";
    
    sdfFile.close();
    
    exportedFiles.push_back(sdfPath.string());
    m_logger.logInfo(std::format("SDF exported to: {}", sdfPath.string()));
    
    return true;
}
