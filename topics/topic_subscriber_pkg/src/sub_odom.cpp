#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class OdomSubscriber : public rclcpp::Node {
public:
  // Initiate a Node called 'simple_subscriber'
  OdomSubscriber() : Node("odom_subscriber") {
    // Create a Subscriber object that will listen to the /counter topic and
    // will call the 'topic_call' function
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "sub_odom", 10, std::bind(&OdomSubscriber::odom_callback, this, _1));
  }

private:
  // Define a function called 'topic_call back' that receives a parameter named
  // 'msg'
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->pose.pose.position.x);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<OdomSubscriber>());
  rclcpp::shutdown();
  return 0;
}