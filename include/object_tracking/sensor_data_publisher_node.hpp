#ifndef SENSOR_DATA_PUBLISHER_NODE_HPP
#define SENSOR_DATA_PUBLISHER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

struct Pose2D
{
  float x;      
  float y;      
  float theta;  
};

class SensorDataPublisher : public rclcpp::Node
{
  public:
    SensorDataPublisher();

    ~SensorDataPublisher() {
        running_ = false;
        robot_poses_thread_.join();
        detections_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Sensor data publisher node has been shut down.");
    }

  private:
    // read json file
    bool read_json(const std::string &file_name, 
      std::vector<std::pair<double, Pose2D>>& robotPoses, 
      std::vector<std::pair<double, std::vector<Pose2D>>>& detections);

    // Create pose msg
    geometry_msgs::msg::Pose create_pose(float x, float y, float theta);

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_robot_poses_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_robot_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_detections_;

    // Async methods to publish data
    void publish_robot_poses();
    void publish_detections();
    
    std::atomic<bool> running_;
    std::thread robot_poses_thread_, detections_thread_;
    std::vector<std::pair<double, Pose2D>> robot_poses_;
    std::vector<std::pair<double, std::vector<Pose2D>>> detections_;   

};

#endif