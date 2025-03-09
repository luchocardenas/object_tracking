#ifndef TRANSFORM_DETECTIONS_HPP
#define TRANSFORM_DETECTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct ConfigParams {
  std::string sensor_data_topic;
  std::string sensor_transformed_topic;
};

class TransformDetections : public rclcpp::Node
{
public:
  TransformDetections();

private:
  bool load_config();

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detections_sub_;

  // Subscribers callbacks
  void robot_poses_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void detections_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;

  // Config parameters
  ConfigParams config_params_;

  geometry_msgs::msg::PoseStamped current_robot_pos_;
  geometry_msgs::msg::PoseArray last_poses_;
};

#endif
