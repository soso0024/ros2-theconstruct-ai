#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cstddef>
#include <iostream>

using namespace std::chrono_literals;
using Twist = geometry_msgs::msg::Twist;

class MoveRobot : public rclcpp::Node {
public:
  MoveRobot() : Node("move_robot") {
    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MoveRobot::move_callback, this));
  }

private:
  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void move_callback() {
    auto message = Twist();
    message.linear.x = 0.2;
    message.angular.z = 0.8;

    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobot>());
  rclcpp::shutdown();
  return 0;
}

/*
user:~$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in
free space broken into its linear and angular parts.
Vector3  linear float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z

*/