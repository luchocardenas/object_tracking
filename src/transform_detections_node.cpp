#include "object_tracking/transform_detections_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

TransformDetections::TransformDetections() : Node("transform_detections")
{
  // Don't call load_config in the constructor, it gives an error of bad_weak_ptr
}

void TransformDetections::init()
{
  // Load configuration from YAML file  
  if (!config_.load_config(shared_from_this())) { 
      return;
  } 
  
  // Initialize subscribers
  robot_poses_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "robot_poses", 10,
    std::bind(&TransformDetections::robot_poses_callback, this, std::placeholders::_1));

  detections_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    config_.get_sensor_data_topic(), 10,
    std::bind(&TransformDetections::detections_callback, this, std::placeholders::_1));

  // Initialize publisher
  pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    config_.get_sensor_transformed_topic(), 10);
   
  RCLCPP_INFO(this->get_logger(), "Transform detections node initialized.");   
}

void TransformDetections::robot_poses_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_robot_pos_ = *msg;
}

void TransformDetections::detections_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  // Clear previous transformed poses and keep the last timestamp
  last_poses_.poses.clear();
  last_poses_.header = msg->header;

  // Transform the incoming PoseArray from "robot" CS to "map" CS (global CS)
  try
  {
    for (const auto& pose : msg->poses)
    {
      // Convert robot pose to tf2 transform
      tf2::Transform tf_robot_to_map;
      tf2::fromMsg(current_robot_pos_.pose, tf_robot_to_map);

      // Convert incoming pose to tf2 transform
      tf2::Transform tf_sensor_to_robot;
      tf2::fromMsg(pose, tf_sensor_to_robot);

      // Transform
      tf2::Transform tf_transformed = tf_robot_to_map * tf_sensor_to_robot;

      // Convert back to geometry_msgs::msg::Pose and store
      geometry_msgs::msg::Pose transformed_pose;
      tf2::toMsg(tf_transformed, transformed_pose);
      last_poses_.poses.push_back(transformed_pose);
    }
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
  }

  // publish the transformed pose
  last_poses_.header.frame_id = "map";
  pub_->publish(last_poses_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransformDetections>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}