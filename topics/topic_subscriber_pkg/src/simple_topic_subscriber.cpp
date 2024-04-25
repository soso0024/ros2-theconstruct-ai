#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node {
public:
  // Initiate a Node called 'simple_subscriber'
  SimpleSubscriber() : Node("simple_subscriber") {
    // Create a Subscriber object that will listen to the /counter topic and
    // will call the 'topic_call' function
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "counter", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
  }

private:
  // Define a function called 'topic_call back' that receives a parameter named
  // 'msg'
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    // Print the value 'data' inside the 'msg' parameter
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}