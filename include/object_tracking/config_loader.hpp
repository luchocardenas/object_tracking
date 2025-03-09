#ifndef CONFIG_LOADER_HPP
#define CONFIG_LOADER_HPP

#include <rclcpp/rclcpp.hpp>

struct ConfigParams {
  std::string sensor_data_topic;
  std::string sensor_transformed_topic;
  std::string tracked_objects_topic;
};

class ConfigLoader {
  public:
    ConfigLoader() = default;

    // Load parameters from a YAML file into the node
    bool load_config(rclcpp::Node::SharedPtr node);

    std::string get_sensor_data_topic() const { return config_params_.sensor_data_topic; }
    std::string get_sensor_transformed_topic() const { return config_params_.sensor_transformed_topic; }
    std::string get_tracked_objects_topic() const { return config_params_.tracked_objects_topic; }

  private:
    // Config parameters
    ConfigParams config_params_;
};

#endif  