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
  
  // Markers to publish the object's IDs
  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracked_object_ids", 10);

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

      // Apply moving average filter
      obj.pose.pose.position.x = obj.x_filter.update(pose.position.x);
      obj.pose.pose.position.y = obj.y_filter.update(pose.position.y);

      // Correct 180 flips
      double prev_yaw = tf2::getYaw(obj.pose.pose.orientation);
      double new_yaw = tf2::getYaw(pose.orientation);
      double stable_yaw = stabilize_yaw(prev_yaw, new_yaw);
      
      // Apply exponential filter for yaw to reduce disturbance from 90 degrees jump
      double filtered_yaw = 0.8 * prev_yaw + 0.2 * stable_yaw;
      tf2::Quaternion quaternion;
      quaternion.setRPY(0, 0, filtered_yaw);
      obj.pose.pose.orientation = tf2::toMsg(quaternion);

      obj.last_seen = now;

      // Increase certainty when the object is detected in the current time
      obj.certainty = std::min(1.0, obj.certainty + certainty_delta); 
    } else {
      // Add new object (start with 0.5 certainty)
      TrackedObject new_obj;
      new_obj.id = assignID();
      new_obj.pose.pose = pose;
      new_obj.pose.header.stamp = now;
      new_obj.last_seen = now;
      new_obj.certainty = 0.5; 
      tracked_objects_[new_obj.id] = new_obj;
    }
  }

  // Reduce certainty if the object is not detected in the current time
  for (auto& [id, obj] : tracked_objects_) {
    if (obj.last_seen != now) {  
      obj.certainty = std::max(0.0, obj.certainty - certainty_delta); ;
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
  // Publish tracked objects as PoseArray
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  // MarkerArray to visualize each object's ID
  visualization_msgs::msg::MarkerArray marker_array;
  
  // Clear all previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  for (const auto& obj : tracked_objects_) {
    msg.poses.push_back(obj.second.pose.pose);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = this->now();
    text_marker.ns = "object_ids";
    text_marker.id = obj.second.id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position = obj.second.pose.pose.position;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    std::ostringstream oss; 
    oss << std::fixed << std::setprecision(2) << obj.second.certainty;
    text_marker.text = std::to_string(obj.second.id) + "\n" + oss.str();
    
    marker_array.markers.push_back(text_marker);
  }

  tracking_publisher_->publish(msg);
  marker_publisher_->publish(marker_array);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectTracker>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
