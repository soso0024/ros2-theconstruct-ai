#include "rclcpp/client.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_srvs/srv/detail/empty__struct.hpp"
#include "std_srvs/srv/empty.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;
using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;

class ServiceClient : public rclcpp::Node {
public:
  ServiceClient() : Node("service_client") {
    client_ = this->create_client<Empty>("moving");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceClient::timer_callback, this));
  }

  bool is_service_done() const { return this->service_done_; }

private:
  rclcpp::Client<Empty>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_;

  void timer_callback() {
    while (!client_->wait_for_service(1s)) {
      if (rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }
    auto request = std::make_shared<Empty::Request>();

    service_done_ = false;
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this, _1));
  }

  void response_callback(rclcpp::Client<Empty>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Result: success");
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress");
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto service_client = std::make_shared<ServiceClient>();
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }
  rclcpp::shutdown();
  return 0;
}

/*

user:~/ros2_ws$ ros2 interface show std_srvs/srv/Empty
---

*/ 