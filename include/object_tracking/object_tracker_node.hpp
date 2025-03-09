#ifndef OBJECT_TRACKER_NODE_HPP
#define OBJECT_TRACKER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "object_tracking/config_loader.hpp"
#include <thread>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct TrackedObject {
  int id;
  geometry_msgs::msg::PoseStamped pose;
  rclcpp::Time last_seen;
  double certainty;
};

class ObjectTracker : public rclcpp::Node
{
  public:
    ObjectTracker();
    void init();

  private:
     
    int assignID();  
    double stabilize_yaw(double prev_yaw, double new_yaw);
    int find_matching_object(const geometry_msgs::msg::Pose& pose, double threshold);

    // ROS2
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr transformed_det_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr tracking_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void transformed_det_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void publish_tracked_objects();

    // Store tracked objects in unordered map
    std::unordered_map<int, TrackedObject> tracked_objects_;
    double timeout_duration;
    double certainty_delta;
    int next_id;

    ConfigLoader config_;

};

#endif