#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

class TestNode : public rclcpp::Node {
public:
  TestNode() : Node("test_node") {
    tf2::Quaternion q(0.0, 0.0, 0.7071, 0.7071);
    double yaw = tf2::getYaw(q);
    RCLCPP_INFO(this->get_logger(), "Yaw: %f", yaw);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}