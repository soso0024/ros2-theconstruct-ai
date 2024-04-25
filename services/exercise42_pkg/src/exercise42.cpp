#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <memory>

// using Empty = std_srvs::srv::Empty;
using SetBool = std_srvs::srv::SetBool;

using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("service_moving") {

    srv_ = create_service<SetBool>(
        "new_moving", std::bind(&ServerNode::moving_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<SetBool>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void moving_callback(const std::shared_ptr<SetBool::Request> request,
                       const std::shared_ptr<SetBool::Response> response) {

    auto message = geometry_msgs::msg::Twist();

    if (request->data == true) {
      message.linear.x = 0.2;
      message.angular.z = -0.2;

      // Set the responce success variable to true
      response->success = true;
      // Set the responce message variable to a string
      response->message = "Turning right right right!";
    } else {
      message.linear.x = 0;
      message.angular.z = 0;

      // Set the response success variable to false
      response->success = false;
      // Set the response message variable to a string
      response->message = "It is time to stop!";
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