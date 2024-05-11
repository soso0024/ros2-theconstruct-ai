#include <chrono>
#include <functional>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

#include "actions_quiz_msg/action/distance.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class QuizClient : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDis = rclcpp_action::ClientGoalHandle<Distance>;

  explicit QuizClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("my_quiz_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Distance>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "distance_as");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&QuizClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;
    this->timer_->cancel();
    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Distance::Goal();
    goal_msg.seconds = 20;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Distance>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&QuizClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&QuizClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&QuizClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Distance>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void goal_response_callback(const GoalHandleDis::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void
  feedback_callback(GoalHandleDis::SharedPtr,
                    const std::shared_ptr<const Distance::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback received -> current_dist: %f\n",
                feedback->current_dist);
  }

  void result_callback(const GoalHandleDis::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was avorted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }
    // c_str()は，std::stringからCスタイルの文字列を取得する際に使用
    RCLCPP_INFO(this->get_logger(), "Result -> status: %s, total_dist: %f",
                result.result->status ? "true" : "false",
                result.result->total_dist);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<QuizClient>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}

/*
user:~/ros2_ws$ ros2 interface show action_quiz_msg/action/Distance
int32 seconds
---
bool status
float64 total_dist
---
float64 current_dist
*/