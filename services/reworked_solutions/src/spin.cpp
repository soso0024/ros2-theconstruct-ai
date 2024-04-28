#include "reworked_services/srv/spin.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include <memory>

using Twist = geometry_msgs::msg::Twist;
using Spin = reworked_services::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class SpinNode : public rclcpp::Node {
public:
  SpinNode() : Node("spin_node") {
    srv_ = create_service<Spin>(
        "rotate", std::bind(&SpinNode::spin_callback, this, _1, _2));
    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;

  void spin_callback(const std::shared_ptr<Spin::Request> request,
                     const std::shared_ptr<Spin::Response> response) {
    auto message = Twist();
    if (request->direction == "right") {
      message.angular.z = -request->angular_velocity;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Turning Right Right Right ...");
    } else if (request->direction == "left") {
      message.angular.z = request->angular_velocity;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Turning Left Left Left ...");
    } else {
      response->success = false;
      RCLCPP_INFO(this->get_logger(), "No Action...");
    }
    publisher_->publish(message);

    sleep(request->time);
    message.angular.z = 0;

    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpinNode>());
  rclcpp::shutdown();
  return 0;
}

/*
ros2 service call /rotate reworked_services/srv/Spin "{direction: 'right',
angular_velocity: 1.0, time: 5}"



user:~/ros2_ws$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular
parts.

Vector3  linear
        float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z

*/