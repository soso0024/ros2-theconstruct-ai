#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using Spin = services_quiz_srv::srv::Spin;
using Twist = geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("services_quiz_server") {
    srv_ = create_service<Spin>(
        "rotate", std::bind(&ServerNode::rotating_callback, this, _1, _2));

    publisher_ = this->create_publisher<Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<Twist>::SharedPtr publisher_;

  void rotating_callback(const std::shared_ptr<Spin::Request> request,
                         const std::shared_ptr<Spin::Response> response) {
    auto message = Twist();

    auto vel = request->angular_velocity;

    if (request->direction == "right") {
      message.angular.z = -vel;
      publisher_->publish(message);

      response->success = true;
    } else if (request->direction == "left") {
      message.angular.z = vel;
      publisher_->publish(message);

      response->success = true;
    } else {
      response->success = false;
    }

    startTimer(request->time);
    message.angular.z = 0;
    publisher_->publish(message);
  }

  void startTimer(int duration) {
    std::this_thread::sleep_for(std::chrono::seconds(duration));
    std::cout << "Finish Moving Time " << duration << " Passed\n";
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}