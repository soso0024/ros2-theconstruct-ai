#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cstddef>
#include <iostream>

using namespace std::chrono_literals;
using std_msgs::msg::Int32;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
class SimplePublisher : public rclcpp::Node {
public:
  SimplePublisher() : Node("simple_publisher"), count_(0) {
    publisher_ = this->create_publisher<Int32>("counter", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SimplePublisher::timer_callback, this));
  }

private:
  rclcpp::Publisher<Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  void timer_callback() {
    auto message = Int32();
    message.data = count_;
    count_ += 10;
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}