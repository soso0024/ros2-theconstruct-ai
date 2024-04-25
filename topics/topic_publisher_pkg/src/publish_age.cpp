#include "custom_interfaces/msg/age.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

#include <chrono>
#include <cstddef>
#include <memory>

using namespace std::chrono_literals;
using Age = custom_interfaces::msg::Age;

class RobotAge : public rclcpp::Node {
public:
  RobotAge() : Node("robot_age_publisher") {
    publisher_ = this->create_publisher<Age>("robot_age", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&RobotAge::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Age>::SharedPtr publisher_;

  void timer_callback() {
    auto message = Age();
    message.years = 1;
    message.months = 10;
    message.days = 300;
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotAge>());
  rclcpp::shutdown();
  return 0;
}