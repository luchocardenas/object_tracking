#ifndef OBJECT_TRACKER_NODE_HPP
#define OBJECT_TRACKER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "object_tracking/config_loader.hpp"
#include <thread>
#include <chrono>
#include <deque>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

using namespace boost::accumulators;

class MovingAverageFilter {
  public:
    MovingAverageFilter(size_t window_size) 
      : acc(tag::rolling_window::window_size = window_size) {}

    double update(double new_value) {
      acc(new_value);
      return rolling_mean(acc);
    }

private:
    accumulator_set<double, features<tag::rolling_mean>, tag::rolling_window> acc;
};

struct TrackedObject {
  int id;
  geometry_msgs::msg::PoseStamped pose;
  rclcpp::Time last_seen;
  double certainty;
  MovingAverageFilter x_filter{60};
  MovingAverageFilter y_filter{60};
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
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