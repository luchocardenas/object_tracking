#include "object_tracking/object_tracker_node.hpp"
#include "std_msgs/msg/int32.hpp"
#include <unordered_map>
#include <cmath>
#include <chrono>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

using namespace std::chrono_literals;

ObjectTracker::ObjectTracker() : Node("object_tracker"), timeout_duration(5.0), certainty_delta(0.01), next_id(1) 
{
  // Don't call load_config in the constructor, it gives an error of bad_weak_ptr
}

void ObjectTracker::init()
{
  // Load configuration from YAML file
  if (!config_.load_config(shared_from_this())) { 
    return;
  } 

  // Subscribe to detections in global CS
  transformed_det_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    config_.get_sensor_transformed_topic(), 10, std::bind(&ObjectTracker::transformed_det_callback, this, std::placeholders::_1));

  // Publish tracked objects as PoseArray
  tracking_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    config_.get_tracked_objects_topic(), 10);

  timer_ = this->create_wall_timer(100ms, std::bind(&ObjectTracker::publish_tracked_objects, this));

  RCLCPP_INFO(this->get_logger(), "Object tracker node initialized.");   
}

int ObjectTracker::assignID() {
  return next_id++;
}

double ObjectTracker::stabilize_yaw(double prev_yaw, double new_yaw){
  // Flip angle 180 if needed 
  if (std::abs((new_yaw - prev_yaw)) > M_PI_2) {
    new_yaw += M_PI;
  }
  
  // Normalize angle between 0 and 2*PI
  while (new_yaw > M_PI) 
    new_yaw -= 2 * M_PI;
  while (new_yaw <= -M_PI) 
    new_yaw += 2 * M_PI;
  
  return new_yaw;
}

int ObjectTracker::find_matching_object(const geometry_msgs::msg::Pose& pose, double threshold = 0.5) {
  for (auto& [id, obj] : tracked_objects_) {
    double dx = obj.pose.pose.position.x - pose.position.x;
    double dy = obj.pose.pose.position.y - pose.position.y;
    // Euclidean distance to find object
    if (std::sqrt(dx * dx + dy * dy) < threshold) {
      return id;
    }
  }
  return -1;
}

void ObjectTracker::transformed_det_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
{
  rclcpp::Time now = this->now();

  for (const auto& pose : msg->poses) {
    int existing_id = find_matching_object(pose);

    if (existing_id != -1) {
      // Update existing object
      TrackedObject& obj = tracked_objects_[existing_id];

      // TODO: try with moving average filter because it can have history of previous poses
      // Apply exponential smoothing filter
      obj.pose.pose.position.x = 0.8 * obj.pose.pose.position.x + 0.2 * pose.position.x;
      obj.pose.pose.position.y = 0.8 * obj.pose.pose.position.y + 0.2 * pose.position.y;

      // Correct 180 flips (sometimes it also flips 90)
      //? Maybe do not change the orientation and just keep the 1st one?
      double prev_yaw = tf2::getYaw(obj.pose.pose.orientation);
      double new_yaw = tf2::getYaw(pose.orientation);
      double stable_yaw = stabilize_yaw(prev_yaw, new_yaw);
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, stable_yaw);
      obj.pose.pose.orientation = tf2::toMsg(quaternion);

      obj.last_seen = now;
      // Increase certainty when the object is detected in the current time
      obj.certainty = std::min(1.0, obj.certainty + certainty_delta); 
    } else {
      // Add new object
      TrackedObject new_obj;
      new_obj.id = assignID();
      new_obj.pose.pose = pose;
      new_obj.pose.header.stamp = now;
      new_obj.last_seen = now;
      // New objects start with 0.5 certainty
      new_obj.certainty = 0.5; 
      tracked_objects_[new_obj.id] = new_obj;
    }
  }

  // Reduce certainty if the object is not detected in the current time
  for (auto& [id, obj] : tracked_objects_) {
    if (obj.last_seen != now) {  
      obj.certainty -= std::max(0.0, obj.certainty - certainty_delta); ;
    }
  }

  // Remove objects that have not been detected for longer period (timeout_duration)
  for (auto it = tracked_objects_.begin(); it != tracked_objects_.end();) {
    if ((now - it->second.last_seen).seconds() > timeout_duration) {
        RCLCPP_INFO(this->get_logger(), "Object '%d' removed due to timeout.", it->second.id);
        it = tracked_objects_.erase(it);
    }
    else {
        ++it;
    }
  }
}

void ObjectTracker::publish_tracked_objects() 
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  for (const auto& obj : tracked_objects_) {
      msg.poses.push_back(obj.second.pose.pose);
  }

  tracking_publisher_->publish(msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectTracker>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
