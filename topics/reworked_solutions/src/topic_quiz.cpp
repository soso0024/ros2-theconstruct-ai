#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cstddef>

using namespace std::chrono_literals;
using std::placeholders::_1;
using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

class AvoidRobot : public rclcpp::Node {
public:
  AvoidRobot() : Node("avoid_robot") {
    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
    subsc_ = this->create_subscription<LaserScan>(
        "scan", 10, std::bind(&AvoidRobot::avoid_callback, this, _1));

    // timer_ = this->create_wall_timer(
    //     500ms, std::bind(&AvoidRobot::avoid_callback, this));
  }

private:
  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  rclcpp::Subscription<LaserScan>::SharedPtr subsc_;
  // rclcpp::TimerBase::SharedPtr timer_;

  void avoid_callback(const LaserScan::SharedPtr msg) {
    size_t size_ranges = msg->ranges.size();

    size_t size_mid_ranges_idx = size_ranges / 2;
    size_t size_right_idx = size_mid_ranges_idx / 2;
    size_t size_left_idx = size_mid_ranges_idx + size_right_idx;

    RCLCPP_INFO(this->get_logger(), "size_ranges: '%ld'", size_right_idx);

    float mid_ranges_val = msg->ranges[size_mid_ranges_idx];
    float left_val = msg->ranges[size_right_idx];
    float right_val = msg->ranges[size_left_idx];

    auto message = Twist();

    if (mid_ranges_val < 1 || right_val < 1) {
      message.angular.z = 5;
    } else if (left_val < 1) {
      message.angular.z = 5;
    } else {
      message.linear.x = 5;
    }

    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidRobot>());
  rclcpp::shutdown();
  return 0;
}

/*

user:~/ros2_ws$ ros2 interface proto sensor_msgs/msg/LaserScan
"header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
angle_min: 0.0
angle_max: 0.0
angle_increment: 0.0
time_increment: 0.0
scan_time: 0.0
range_min: 0.0
range_max: 0.0
ranges: []
intensities: []
"

*/