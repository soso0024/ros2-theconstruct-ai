#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sofar/msg/age.hpp"

using namespace std::chrono_literals;
using Age = sofar::msg::Age;

class AgeRobot : public rclcpp::Node {
public:
  AgeRobot() : Node("age_robot") {
    publisher_ = this->create_publisher<Age>("age", 10);
    timer_ =
        this->create_wall_timer(500ms, std::bind(&AgeRobot::publish_age, this));
  }

private:
  rclcpp::Publisher<Age>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void publish_age() {
    auto message = Age();
    message.years = 1;
    message.months = 3;
    message.days = 10;

    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgeRobot>());
  rclcpp::shutdown();
  return 0;
}