#include "object_tracking/sensor_data_publisher_node.hpp"
#include <memory>
#include <fstream>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>

using json = nlohmann::json;

SensorDataPublisher::SensorDataPublisher() : Node("sensor_data_publisher")
{
  // Load json data once during initialization
  if (!read_json("data", robot_poses_, detections_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read json data.");
    return;
  }

  // Create publishers
  pub_robot_poses_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_poses", 10);
  pub_robot_path_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/robot_path", 10);
  pub_detections_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detections", 10);
  
  // Start async publishing
  running_ = true;
  robot_poses_thread_ = std::thread(&SensorDataPublisher::publish_robot_poses, this);
  detections_thread_ = std::thread(&SensorDataPublisher::publish_detections, this);

  // Publish robot path once for visualization
  auto msg_rp = geometry_msgs::msg::PoseArray();
  msg_rp.header.frame_id = "map";
  for(const auto& robot_pose : robot_poses_)
  {
    msg_rp.poses.push_back(create_pose(robot_pose.second.x, robot_pose.second.y, robot_pose.second.theta));
  }
  pub_robot_path_->publish(msg_rp);

  RCLCPP_INFO(this->get_logger(), "Sensor data publisher node initialized.");
}

bool SensorDataPublisher::read_json(const std::string &file_name, 
  std::vector<std::pair<double, Pose2D>>& robotPoses, 
  std::vector<std::pair<double, std::vector<Pose2D>>>& detections)
{
  // Crete file_path to read the json data
  std::string package_path = ament_index_cpp::get_package_share_directory("object_tracking");
  std::string json_file_path = package_path + "/data/" + file_name + ".json";
  try {      
    std::ifstream inputStream(json_file_path);
    if (!inputStream.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open JSON file: %s", json_file_path.c_str());
      return false;
    }

    json jsonFile;
    inputStream >> jsonFile;
    
    for (const auto& pose: jsonFile["robotPose"])
      robotPoses.emplace_back(
        std::make_pair(pose["time"].get<double>(),
                       Pose2D{pose["x"].get<float>(),
                              pose["y"].get<float>(),
                              pose["theta"].get<float>()}));
    RCLCPP_INFO(this->get_logger(), "Successfully parsed %lu robot poses.", robotPoses.size());

    for (const auto& detection: jsonFile["detections"])
    {
      std::vector<Pose2D> poses;
      for (const auto& object: detection["poses"])
        poses.emplace_back(Pose2D {object["x"].get<float>(),
                                   object["y"].get<float>(),
                                   object["theta"].get<float>()});
      detections.emplace_back(std::make_pair(detection["time"].get<double>(), poses));
    }
    RCLCPP_INFO(this->get_logger(), "Successfully parsed %lu detections.", detections.size());

    return true;
    }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON data: %s", e.what());
    return false;
  }
}

geometry_msgs::msg::Pose SensorDataPublisher::create_pose(float x, float y, float theta)
{
  // Pose msg for visualization in Rviz2 
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, theta);
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  return pose;
}

void SensorDataPublisher::publish_robot_poses()
{
  // Transform each Pose2D to PoseStamped and publish it
  
  // Store the first time stamp as reference start time
  rclcpp::Time t0 = rclcpp::Time(static_cast<int64_t>(robot_poses_.at(0).first * 1e9));
  rclcpp::Time current_t;
  auto start_time = std::chrono::steady_clock::now();

  size_t index = 0;
  while (rclcpp::ok() && running_ && (index < robot_poses_.size()))
  {
    // Calculate elapsed time since start in ROS time units
    current_t = rclcpp::Time(static_cast<int64_t>(robot_poses_.at(index).first * 1e9));
    auto now = std::chrono::steady_clock::now();
    double elapsed_time = std::chrono::duration<double>(now - start_time).count();

    // Convert pose timestamp to elapsed time relative to t0
    rclcpp::Duration time_since_t0 = current_t - t0;
    double target_time = time_since_t0.seconds();

    if (elapsed_time >= target_time)
      {
        // Create PoseStamped msg and publish it
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = current_t;
        msg.header.frame_id = "map";
        msg.pose = create_pose(robot_poses_.at(index).second.x, robot_poses_.at(index).second.y, robot_poses_.at(index).second.theta);
        pub_robot_poses_->publish(msg);
        index++;
      }
    }
}

void SensorDataPublisher::publish_detections()
{
  // Transform each detection to PoseArray and publish it
  // The detections are in robot CS
  
  // Store the first time stamp as reference start time
  rclcpp::Time t0 = rclcpp::Time(static_cast<int64_t>(detections_.at(0).first * 1e9));
  rclcpp::Time current_t;
  auto start_time = std::chrono::steady_clock::now();
  
  size_t index = 0;
  while (rclcpp::ok() && running_ && (index < detections_.size()))
  {
    // Calculate elapsed time since start in ROS time
    current_t = rclcpp::Time(static_cast<int64_t>(detections_.at(index).first * 1e9));
    auto now = std::chrono::steady_clock::now();
    double elapsed_time = std::chrono::duration<double>(now - start_time).count();

    // Convert pose timestamp to elapsed time relative to t0
    rclcpp::Duration time_since_t0 = current_t - t0;
    double target_time = time_since_t0.seconds();

    if (elapsed_time >= target_time)
      {
        // Create PoseArray msg and publish it
        auto msg = geometry_msgs::msg::PoseArray();
        msg.header.stamp = current_t;
        msg.header.frame_id = "robot";

        for(const auto& det_pose : detections_.at(index).second)
          msg.poses.push_back(create_pose(det_pose.x, det_pose.y, det_pose.theta));

        pub_detections_->publish(msg);
        index++;
      }
    } 
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorDataPublisher>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}