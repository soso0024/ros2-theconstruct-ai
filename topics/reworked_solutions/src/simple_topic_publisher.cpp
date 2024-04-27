#include "rclcpp/executors.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include "std_msgs/msg/int32.hpp"
#include <memory>

using std_msgs::msg::Int32;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_publishe");
  auto publisher = node->create_publisher<Int32>("counter", 10);
  auto message = std::make_shared<Int32>();
  message->data = 0;
  rclcpp::WallRate loop_rate(2);

  while (rclcpp::ok()) {
    publisher->publish(*message);
    message->data++;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}