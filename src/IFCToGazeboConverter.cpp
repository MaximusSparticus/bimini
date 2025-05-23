#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>
#include <string>

#include "bimini/BIMInterface.hpp"
#include "bimini_msgs/srv/export_simulation_model.hpp"

class IFCToGazeboConverter : public rclcpp::Node {
public:
    IFCToGazeboConverter() : Node("ifc_to_gazebo_converter") {
        // Declare parameters
        this->declare_parameter<std::string>("ifc_file", "construction_site.ifc");
        this->declare_parameter<std::string>("output_dir", "generated_worlds");
        this->declare_parameter<bool>("simplify_geometry", true);
        this->declare_parameter<double>("wall_height_default", 3.0);
        this->declare_parameter<double>("wall_thickness_default", 0.2);
        
        // Get parameters
        ifc_file_ = this->get_parameter("ifc_file").as_string();
        output_dir_ = this->get_parameter("output_dir").as_string();
        simplify_geometry_ = this->get_parameter("simplify_geometry").as_bool();
        
        RCLCPP_INFO(this->get_logger(), "IFC to Gazebo Converter initialized");
        RCLCPP_INFO(this->get_logger(), "Input IFC file: %s", ifc_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output directory: %s", output_dir_.c_str());
    }
    
    bool convertIFCToGazebo() {
        // Load IFC file using bimini
        auto bim_interface = std::make_shared<bimini::BIMInterface>();
        
        if (!bim_interface->loadIFC(ifc_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load IFC file: %s", ifc_file_.c_str());
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "IFC file loaded successfully");
        
        // Create output directory
        std::filesystem::create_directories(output_dir_);
        
        // Generate SDF world file
        std::string world_file = generateGazeboWorld(bim_interface);
        
        if (world_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate Gazebo world");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Gazebo world generated: %s", world_file.c_str());
        
        // Generate launch file
        std::string launch_file = generateLaunchFile(world_file);
        RCLCPP_INFO(this->get_logger(), "Launch file generated: %s", launch_file.c_str());
        
        return true;
    }

private:
    std::string generateGazeboWorld(std::shared_ptr<bimini::BIMInterface> bim_interface) {
        std::stringstream sdf;
        
        // Start SDF world
        sdf << "<?xml version=\"1.0\"?>\n";
        sdf << "<sdf version=\"1.8\">\n";
        sdf << "  <world name=\"construction_site_from_ifc\">\n";
        
        // Add lighting
        sdf << "    <!-- Lighting -->\n";
        sdf << "    <light type=\"directional\" name=\"sun\">\n";
        sdf << "      <cast_shadows>true</cast_shadows>\n";
        sdf << "      <pose>0 0 10 0 0 0</pose>\n";
        sdf << "      <diffuse>0.8 0.8 0.8 1</diffuse>\n";
        sdf << "      <specular>0.2 0.2 0.2 1</specular>\n";
        sdf << "      <direction>-0.5 0.1 -0.9</direction>\n";
        sdf << "    </light>\n\n";
        
        // Add ground plane
        sdf << "    <!-- Ground Plane -->\n";
        sdf << "    <model name=\"ground_plane\">\n";
        sdf << "      <static>true</static>\n";
        sdf << "      <link name=\"link\">\n";
        sdf << "        <collision name=\"collision\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <plane>\n";
        sdf << "              <normal>0 0 1</normal>\n";
        sdf << "              <size>100 100</size>\n";
        sdf << "            </plane>\n";
        sdf << "          </geometry>\n";
        sdf << "        </collision>\n";
        sdf << "        <visual name=\"visual\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <plane>\n";
        sdf << "              <normal>0 0 1</normal>\n";
        sdf << "              <size>100 100</size>\n";
        sdf << "            </plane>\n";
        sdf << "          </geometry>\n";
        sdf << "          <material>\n";
        sdf << "            <ambient>0.8 0.7 0.6 1</ambient>\n";
        sdf << "            <diffuse>0.8 0.7 0.6 1</diffuse>\n";
        sdf << "          </material>\n";
        sdf << "        </visual>\n";
        sdf << "      </link>\n";
        sdf << "    </model>\n\n";
        
        // Process IFC entities and convert to SDF models
        // NOTE: This is a simplified conversion. In practice, you would need to:
        // 1. Parse the actual geometry from IFC
        // 2. Convert IFC coordinate system to Gazebo coordinate system
        // 3. Handle different IFC element types appropriately
        
        // For now, let's create some placeholder construction elements
        sdf << "    <!-- Construction Site Elements from IFC -->\n";
        
        // Add walls as box models (simplified)
        addWallModel(sdf, "south_wall", 10, 0, 1.25, 20, 0.2, 2.5);
        addWallModel(sdf, "east_wall", 20, 7.5, 1.5, 0.2, 15, 3.0);
        addWallModel(sdf, "north_wall_section1", 15, 15, 1.5, 10, 0.2, 3.0);
        addWallModel(sdf, "west_wall", 0, 7.5, 0.75, 0.2, 15, 1.5);
        
        // Add columns
        addColumnModel(sdf, "column_1", 5, 5, 1.5, 0.3, 0.3, 3.0);
        addColumnModel(sdf, "column_2", 10, 5, 1.5, 0.3, 0.3, 3.0);
        addColumnModel(sdf, "column_3", 15, 5, 1.5, 0.3, 0.3, 3.0);
        addColumnModel(sdf, "column_4", 5, 10, 1.5, 0.3, 0.3, 3.0);
        addColumnModel(sdf, "column_5", 10, 10, 1.5, 0.3, 0.3, 3.0);
        addColumnModel(sdf, "column_6", 15, 10, 1.5, 0.3, 0.3, 3.0);
        
        // Add construction equipment placeholder
        addConstructionEquipment(sdf);
        
        // Add construction barriers
        addBarrierModel(sdf, "barrier_1", 10, -2, 0.5, 24, 0.1, 1.0);
        addBarrierModel(sdf, "barrier_2", 22, 7.5, 0.5, 0.1, 19, 1.0);
        
        // Add material storage area
        addStorageArea(sdf, "material_storage", 10, 19.5, 0.1, 10, 5, 0.2);
        
        // Physics settings
        sdf << "    <!-- Physics Settings -->\n";
        sdf << "    <physics type=\"ode\">\n";
        sdf << "      <ode>\n";
        sdf << "        <solver>\n";
        sdf << "          <type>quick</type>\n";
        sdf << "          <iters>50</iters>\n";
        sdf << "          <sor>1.4</sor>\n";
        sdf << "        </solver>\n";
        sdf << "        <constraints>\n";
        sdf << "          <cfm>0.0</cfm>\n";
        sdf << "          <erp>0.2</erp>\n";
        sdf << "          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>\n";
        sdf << "          <contact_surface_layer>0.001</contact_surface_layer>\n";
        sdf << "        </constraints>\n";
        sdf << "      </ode>\n";
        sdf << "    </physics>\n\n";
        
        // Scene settings
        sdf << "    <!-- Scene Settings -->\n";
        sdf << "    <scene>\n";
        sdf << "      <ambient>0.7 0.7 0.7 1</ambient>\n";
        sdf << "      <background>0.8 0.8 1 1</background>\n";
        sdf << "      <shadows>true</shadows>\n";
        sdf << "    </scene>\n";
        
        // Close world
        sdf << "  </world>\n";
        sdf << "</sdf>\n";
        
        // Write to file
        std::string world_filename = output_dir_ + "/construction_site_from_ifc.sdf";
        std::ofstream world_file(world_filename);
        world_file << sdf.str();
        world_file.close();
        
        return world_filename;
    }
    
    void addWallModel(std::stringstream& sdf, const std::string& name, 
                      double x, double y, double z,
                      double sx, double sy, double sz) {
        sdf << "    <model name=\"" << name << "\">\n";
        sdf << "      <static>true</static>\n";
        sdf << "      <pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n";
        sdf << "      <link name=\"link\">\n";
        sdf << "        <collision name=\"collision\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <box>\n";
        sdf << "              <size>" << sx << " " << sy << " " << sz << "</size>\n";
        sdf << "            </box>\n";
        sdf << "          </geometry>\n";
        sdf << "        </collision>\n";
        sdf << "        <visual name=\"visual\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <box>\n";
        sdf << "              <size>" << sx << " " << sy << " " << sz << "</size>\n";
        sdf << "            </box>\n";
        sdf << "          </geometry>\n";
        sdf << "          <material>\n";
        sdf << "            <ambient>0.6 0.6 0.6 1</ambient>\n";
        sdf << "            <diffuse>0.7 0.7 0.7 1</diffuse>\n";
        sdf << "          </material>\n";
        sdf << "        </visual>\n";
        sdf << "      </link>\n";
        sdf << "    </model>\n\n";
    }
    
    void addColumnModel(std::stringstream& sdf, const std::string& name,
                        double x, double y, double z,
                        double sx, double sy, double sz) {
        sdf << "    <model name=\"" << name << "\">\n";
        sdf << "      <static>true</static>\n";
        sdf << "      <pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n";
        sdf << "      <link name=\"link\">\n";
        sdf << "        <collision name=\"collision\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <box>\n";
        sdf << "              <size>" << sx << " " << sy << " " << sz << "</size>\n";
        sdf << "            </box>\n";
        sdf << "          </geometry>\n";
        sdf << "        </collision>\n";
        sdf << "        <visual name=\"visual\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <box>\n";
        sdf << "              <size>" << sx << " " << sy << " " << sz << "</size>\n";
        sdf << "            </box>\n";
        sdf << "          </geometry>\n";
        sdf << "          <material>\n";
        sdf << "            <ambient>0.5 0.5 0.5 1</ambient>\n";
        sdf << "            <diffuse>0.6 0.6 0.6 1</diffuse>\n";
        sdf << "          </material>\n";
        sdf << "        </visual>\n";
        sdf << "      </link>\n";
        sdf << "    </model>\n\n";
    }
    
    void addBarrierModel(std::stringstream& sdf, const std::string& name,
                         double x, double y, double z,
                         double sx, double sy, double sz) {
        sdf << "    <model name=\"" << name << "\">\n";
        sdf << "      <static>true</static>\n";
        sdf << "      <pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n";
        sdf << "      <link name=\"link\">\n";
        sdf << "        <collision name=\"collision\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <box>\n";
        sdf << "              <size>" << sx << " " << sy << " " << sz << "</size>\n";
        sdf << "            </box>\n";
        sdf << "          </geometry>\n";
        sdf << "        </collision>\n";
        sdf << "        <visual name=\"visual\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <box>\n";
        sdf << "              <size>" << sx << " " << sy << " " << sz << "</size>\n";
        sdf << "            </box>\n";
        sdf << "          </geometry>\n";
        sdf << "          <material>\n";
        sdf << "            <ambient>1.0 0.5 0.0 1</ambient>\n";
        sdf << "            <diffuse>1.0 0.5 0.0 1</diffuse>\n";
        sdf << "          </material>\n";
        sdf << "        </visual>\n";
        sdf << "      </link>\n";
        sdf << "    </model>\n\n";
    }
    
    void addStorageArea(std::stringstream& sdf, const std::string& name,
                        double x, double y, double z,
                        double sx, double sy, double sz) {
        sdf << "    <model name=\"" << name << "\">\n";
        sdf << "      <static>true</static>\n";
        sdf << "      <pose>" << x << " " << y << " " << z << " 0 0 0</pose>\n";
        sdf << "      <link name=\"link\">\n";
        sdf << "        <collision name=\"collision\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <box>\n";
        sdf << "              <size>" << sx << " " << sy << " " << sz << "</size>\n";
        sdf << "            </box>\n";
        sdf << "          </geometry>\n";
        sdf << "        </collision>\n";
        sdf << "        <visual name=\"visual\">\n";
        sdf << "          <geometry>\n";
        sdf << "            <box>\n";
        sdf << "              <size>" << sx << " " << sy << " " << sz << "</size>\n";
        sdf << "            </box>\n";
        sdf << "          </geometry>\n";
        sdf << "          <material>\n";
        sdf << "            <ambient>0.4 0.4 0.4 1</ambient>\n";
        sdf << "            <diffuse>0.5 0.5 0.5 1</diffuse>\n";
        sdf << "          </material>\n";
        sdf << "        </visual>\n";
        sdf << "      </link>\n";
        sdf << "    </model>\n\n";
    }
    
    void addConstructionEquipment(std::stringstream& sdf) {
        // Add some construction cones
        sdf << "    <include>\n";
        sdf << "      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/construction_cone</uri>\n";
        sdf << "      <name>cone_1</name>\n";
        sdf << "      <pose>3 -4 0 0 0 0</pose>\n";
        sdf << "    </include>\n\n";
        
        sdf << "    <include>\n";
        sdf << "      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/construction_cone</uri>\n";
        sdf << "      <name>cone_2</name>\n";
        sdf << "      <pose>17 -4 0 0 0 0</pose>\n";
        sdf << "    </include>\n\n";
        
        // Add construction barrel
        sdf << "    <include>\n";
        sdf << "      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/construction_barrel</uri>\n";
        sdf << "      <name>barrel_1</name>\n";
        sdf << "      <pose>10 17 0 0 0 0</pose>\n";
        sdf << "    </include>\n\n";
    }
    
    std::string generateLaunchFile(const std::string& world_file) {
        std::stringstream launch;
        
        launch << "import os\n";
        launch << "from ament_index_python.packages import get_package_share_directory\n";
        launch << "from launch import LaunchDescription\n";
        launch << "from launch.actions import ExecuteProcess, IncludeLaunchDescription\n";
        launch << "from launch.launch_description_sources import PythonLaunchDescriptionSource\n";
        launch << "from launch_ros.actions import Node\n\n";
        
        launch << "def generate_launch_description():\n";
        launch << "    # Get package directories\n";
        launch << "    pkg_share = get_package_share_directory('bimini')\n";
        launch << "    bimbot_share = get_package_share_directory('bimbot')\n\n";
        
        launch << "    # Path to world file\n";
        launch << "    world_file = '" << world_file << "'\n\n";
        
        launch << "    # Launch Gazebo with the generated world\n";
        launch << "    gz_sim = ExecuteProcess(\n";
        launch << "        cmd=['gz', 'sim', '-r', world_file],\n";
        launch << "        output='screen'\n";
        launch << "    )\n\n";
        
        launch << "    # Include robot spawning launch file\n";
        launch << "    spawn_robots = IncludeLaunchDescription(\n";
        launch << "        PythonLaunchDescriptionSource(\n";
        launch << "            os.path.join(bimbot_share, 'launch', 'spawn_robots.launch.py')\n";
        launch << "        ),\n";
        launch << "        launch_arguments={\n";
        launch << "            'world': world_file,\n";
        launch << "            'spawn_builderbot': 'true',\n";
        launch << "            'spawn_inspectorbot': 'true'\n";
        launch << "        }.items()\n";
        launch << "    )\n\n";
        
        launch << "    return LaunchDescription([\n";
        launch << "        gz_sim,\n";
        launch << "        spawn_robots\n";
        launch << "    ])\n";
        
        // Write launch file
        std::string launch_filename = output_dir_ + "/construction_site_sim.launch.py";
        std::ofstream launch_file(launch_filename);
        launch_file << launch.str();
        launch_file.close();
        
        return launch_filename;
    }
    
private:
    std::string ifc_file_;
    std::string output_dir_;
    bool simplify_geometry_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto converter = std::make_shared<IFCToGazeboConverter>();
    
    if (converter->convertIFCToGazebo()) {
        RCLCPP_INFO(converter->get_logger(), "Conversion completed successfully!");
    } else {
        RCLCPP_ERROR(converter->get_logger(), "Conversion failed!");
    }
    
    rclcpp::shutdown();
    return 0;
}
