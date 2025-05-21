#include <rclcpp/rclcpp.hpp>
#include <bimini_msgs/srv/load_building.hpp>
#include <memory>

class BiminiClient : public rclcpp::Node {
public:
    BiminiClient() : Node("bimini_client") {
        // Create a service client
        client_ = create_client<bimini_msgs::srv::LoadBuilding>("load_building");
        
        // Wait for service to become available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
        }
        
        // Now that service is available, send a request
        sendRequest();
    }

private:
    void sendRequest() {
        auto request = std::make_shared<bimini_msgs::srv::LoadBuilding::Request>();
        request->file_path = "/path/to/building.ifc";
        request->validate_file = true;
        
        RCLCPP_INFO(this->get_logger(), "Sending request to load building from: %s", 
                   request->file_path.c_str());
                   
        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<bimini_msgs::srv::LoadBuilding>::SharedFuture future) {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Building loaded successfully with ID: %s", 
                               response->building_id.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load building: %s", 
                                response->error_message.c_str());
                }
            }
        );
    }

    rclcpp::Client<bimini_msgs::srv::LoadBuilding>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BiminiClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
