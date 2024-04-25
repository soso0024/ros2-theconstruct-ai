#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cstddef>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TopicsQuiz : public rclcpp::Node {
public:
  TopicsQuiz() : Node("topics_quiz_node") {

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&TopicsQuiz::detect_front, this, _1));

    timer_ = this->create_wall_timer(
        500ms, std::bind(&TopicsQuiz::control_robot, this));
  }

private:
  void control_robot() {
    auto message = geometry_msgs::msg::Twist();

    // initialize
    message.linear.x = 0;
    message.angular.z = 0;

    if (front_value < 1 || right_value < 1) {
      message.angular.z = 0.3;
    }

    else if (left_value < 1) {
      message.angular.z = -0.3;
    }

    else {
      message.linear.x = 0.3;
    }

    publisher_->publish(message);
  }

  void detect_front(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int range_size = msg->ranges.size();
    int angle30_index = (range_size - 1) * 30 / 180;

    int front_index = (range_size - 1) / 2;
    front_value = msg->ranges[front_index];

    right_value = msg->ranges[range_size - angle30_index];

    left_value = msg->ranges[angle30_index];
    
    // FOR CHECK EACH VALUES
    // RCLCPP_INFO(this->get_logger(), "range value: '%ld'", msg.ranges.size());
    // RCLCPP_INFO(this->get_logger(), "Front value: '%f'", front_value);
    // RCLCPP_INFO(this->get_logger(), "Right value: '%f'", right_value);
    // RCLCPP_INFO(this->get_logger(), "Left value: '%f'\n\n", left_value);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  float front_value;
  float right_value;
  float left_value;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<TopicsQuiz>());
  rclcpp::shutdown();
  return 0;
}
