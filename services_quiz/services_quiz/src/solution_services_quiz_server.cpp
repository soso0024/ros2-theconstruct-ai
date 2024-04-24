#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"

#include <memory>

using namespace std::chrono_literals;
using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("services_quiz_server_node") {

    srv_ = create_service<Spin>(
        "rotate", std::bind(&ServerNode::rotate_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist twist_msg;

  void rotate_callback(const std::shared_ptr<Spin::Request> request,
                       const std::shared_ptr<Spin::Response> response) {

    if (request->direction == "right") {
      this->twist_msg.angular.z = -request->angular_velocity;
    } else {
      this->twist_msg.angular.z = request->angular_velocity;
    }

    for (int i = 0; i <= request->time; i++) {
      this->publisher_->publish(this->twist_msg);
      std::this_thread::sleep_for(1000ms);
    }

    // set velocity to zero to stop the robot
    this->twist_msg.angular.z = 0.0;
    this->publisher_->publish(this->twist_msg);

    response->success = true;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}