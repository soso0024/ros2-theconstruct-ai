#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <functional>
#include <memory>

using SetBool = std_srvs::srv::SetBool;
using Twist = geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("service_right") {
    srv_ = create_service<SetBool>(
        "right", std::bind(&ServerNode::right_callback, this, _1, _2));
    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<SetBool>::SharedPtr srv_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;

  void right_callback(const std::shared_ptr<SetBool::Request> request,
                      const std::shared_ptr<SetBool::Response> responce) {
    auto message = Twist();
    if (request->data) {
      message.linear.x = 0.2;
      message.angular.z = -0.2;

      responce->success = true;
      responce->message = "Nice Turn!";
    } else {
      message.linear.x = 0;
      message.angular.z = 0;

      responce->success = false;
      responce->message = "Love Stop";
    }

    if (responce->success) {
      RCLCPP_INFO(this->get_logger(), "Turning Right Right Right ...");
    } else {
      RCLCPP_INFO(this->get_logger(), "Stop!");
    }

    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}

/*

user:~/ros2_ws$ ros2 interface show std_srvs/srv/SetBool
bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages



user:~/ros2_ws$ ros2 interface proto std_srvs/srv/SetBool
"data: false
"



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