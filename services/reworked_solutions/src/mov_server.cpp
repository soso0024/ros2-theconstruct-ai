#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/utilities.hpp"
#include "reworked_services/srv/my_custom_service_message.hpp"
#include <functional>
#include <memory>

using MCSM = reworked_services::srv::MyCustomServiceMessage;
using Twist = geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("movement_server") {
    srv_ = create_service<MCSM>(
        "movement", std::bind(&ServerNode::moving_callback, this, _1, _2));
    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<MCSM>::SharedPtr srv_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;

  void moving_callback(const std::shared_ptr<MCSM::Request> request,
                       const std::shared_ptr<MCSM::Response> response) {
    auto message = Twist();

    if (request->move == "Turn Right") {
      // Send velocities to move the robot to the right
      message.linear.x = 0.2;
      message.angular.z = -0.2;
      publisher_->publish(message);

      // Set the response success variable to true
      response->success = true;
    } else if (request->move == "Turn Left") {
      // Send velocities to stop the robot
      message.linear.x = 0.2;
      message.angular.z = 0.2;
      publisher_->publish(message);

      // Set the response success variable to false
      response->success = true;
    } else if (request->move == "Stop") {
      // Send velocities to stop the robot
      message.linear.x = 0.0;
      message.angular.z = 0.0;
      publisher_->publish(message);

      // Set the response success variable to false
      response->success = true;
    } else {
      response->success = false;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}

/*
user:~/ros2_ws$ ros2 interface show reworked_services/srv/MyCustomServiceMessage
string move   # Signal to define the movement
              # "Turn right" to make the robot turn in the right direction.
              # "Turn left" to make the robot turn in the left direction.
              # "Stop" to make the robot stop the movement.

---
bool success  # Did it achieve it?



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



user:~/ros2_ws$ ros2 service call /movement
reworked_services/srv/MyCustomServiceMessage move:\ "Turn Left" waiting for
service to become available... requester: making request:
reworked_services.srv.MyCustomServiceMessage_Request(move='Turn Left')

response:
reworked_services.srv.MyCustomServiceMessage_Response(success=True)
*/