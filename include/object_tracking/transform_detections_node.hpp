#ifndef TRANSFORM_DETECTIONS_HPP
#define TRANSFORM_DETECTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include "object_tracking/config_loader.hpp"
#include <memory>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class TransformDetections : public rclcpp::Node
{
public:
  TransformDetections();
  void init();

private:

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr detections_sub_;

  // Subscribers callbacks
  void robot_poses_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void detections_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;

  geometry_msgs::msg::PoseStamped current_robot_pos_;
  geometry_msgs::msg::PoseArray last_poses_;

  ConfigLoader config_;
};

#endif
