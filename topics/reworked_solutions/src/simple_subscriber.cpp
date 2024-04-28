#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include "std_msgs/msg/int32.hpp"

using geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std_msgs::msg::Int32;

class SimpleSubscriber : public rclcpp::Node {
public:
  SimpleSubscriber() : Node("simple_subscriber") {
    // subscription_ = this->create_subscription<Int32>(
    //     "counter", 10, std::bind(&SimpleSubscriber::topic_callback, this,
    //     _1));

    subscription_ = this->create_subscription<Twist>(
        "cmd_vel", 10, std::bind(&SimpleSubscriber::vel_callback, this, _1));
  }

private:
  // void topic_callback(const Int32::SharedPtr msg) {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
  // }

  void vel_callback(const Twist::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "x vel: '%f'", msg->linear.x);
  }
  // rclcpp::Subscription<Int32>::SharedPtr subscription_;
  rclcpp::Subscription<Twist>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
