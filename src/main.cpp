#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bimini_msgs/msg/building_model.hpp"
#include "bimini_msgs/srv/load_building.hpp"

/**
 * @brief BiminiNode is a ROS node which provides an interface to a BIM model to other ROS nodes
 * @todo comments
 * @todo define message interfaces
 */
class BiminiNode : public rclcpp::Node {
 public:
    BiminiNode()
    : Node("bimini")
    , m_timer()
    , m_service()
    , m_publisher()
    , m_subscriber()
    , m_lastMsg() {
        using namespace std::chrono_literals;

        // Create services
        m_service = create_service<bimini_msgs::srv::LoadBuilding>( "load_building",
            std::bind(&BiminiNode::loadBuildingCallback, this,
            std::placeholders::_1, std::placeholders::_2));

        // Initialize publisher
        m_publisher = this->create_publisher<bimini_msgs::msg::BuildingModel>("building_model", 10);
    
        // Initialize timer for publishing at regular intervals
        m_timer = this->create_wall_timer(1000ms, std::bind(&BiminiNode::timer_callback, this));
    
        // Initialize subscriber
        m_subscriber = this->create_subscription<std_msgs::msg::String>("input_topic", 10,
            std::bind(&BiminiNode::topic_callback, this, std::placeholders::_1));
    
        RCLCPP_INFO(this->get_logger(), "Node initialized");
    }

 private:
    // Service callback function
    void loadBuildingCallback(
        const std::shared_ptr<bimini_msgs::srv::LoadBuilding::Request> request,
        std::shared_ptr<bimini_msgs::srv::LoadBuilding::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received request to load building: %s", 
                   request->file_path.c_str());
                   
        try {
            // Import IFC file using your Bimini library
            // auto building = importer_.importIFC(request->file_path);
            
            // Generate a unique ID for the building
            //std::string buildingId = generateUniqueId();
            std::string buildingId = "todo - should be unique";
            
            // Store the building in your node's state or database
            // buildings_[buildingId] = building;
            
            // Set response
            response->success = true;
            response->building_id = buildingId;
            
            RCLCPP_INFO(this->get_logger(), "Building loaded successfully with ID: %s", 
                       buildingId.c_str());
        }
        catch (const std::exception& e) {
            response->success = false;
            response->error_message = "Failed to load building: " + std::string(e.what());
            
            RCLCPP_ERROR(this->get_logger(), "Failed to load building: %s", e.what());
        }
    }
    
    void timer_callback() {
        auto msg = bimini_msgs::msg::BuildingModel();
        msg.header.stamp = this->now();
        msg.header.frame_id = "world";
        msg.building_id = "building_001";
        msg.name = "Office Building";
        msg.source_file = "some_path.ifc";
        RCLCPP_INFO(this->get_logger(), std::format(
            "msg.header.stamp = (), msg.header.frame_id = {}, msg.building_id = {}, msg.name = {}, msg.source_file = {}",
            //msg.header.stamp,
            msg.header.frame_id,
            msg.building_id,
            msg.name,
            msg.source_file).c_str());
        m_publisher->publish(msg);
    }
  
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    
        // Example of processing received data
        m_lastMsg = msg->data;
    }
  
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Service<bimini_msgs::srv::LoadBuilding>::SharedPtr m_service;
    rclcpp::Publisher<bimini_msgs::msg::BuildingModel>::SharedPtr m_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscriber;
    std::string m_lastMsg;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
  
    // Create and spin the node
    auto node = std::make_shared<BiminiNode>();
  
    // Process callbacks until the user manually kills the process
    rclcpp::spin(node);
  
    // Shutdown and exit
    rclcpp::shutdown();
    return 0;
}
