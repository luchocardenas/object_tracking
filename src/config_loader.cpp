#include "object_tracking/config_loader.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

bool ConfigLoader::load_config(rclcpp::Node::SharedPtr node)
{  
  if (!node) {
    RCLCPP_ERROR(node->get_logger(), "Node pointer is null.");
    return false;
  }

  std::string package_path = ament_index_cpp::get_package_share_directory("object_tracking");  
  std::string config_file_path = package_path + "/config/sensors.yaml";
  try {
    YAML::Node config = YAML::LoadFile(config_file_path);

    // Read parameters from YAML
    config_params_.sensor_data_topic = config["sensor_data_topic"].as<std::string>();
    config_params_.sensor_transformed_topic = config["sensor_transformed_topic"].as<std::string>();
    config_params_.tracked_objects_topic = config["tracked_objects_topic"].as<std::string>();

    RCLCPP_INFO(node->get_logger(), "sensor_data_topic=%s", config_params_.sensor_data_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "sensor_transformed_topic=%s", config_params_.sensor_transformed_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "tracked_objects_topic=%s", config_params_.tracked_objects_topic.c_str());
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load config at path: %s ; Error: %s", config_file_path.c_str(), e.what());
    return false;
  }
}