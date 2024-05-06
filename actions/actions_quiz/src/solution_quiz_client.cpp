#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>

#include "actions_quiz_msg/action/distance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ActionsQuizClient : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ClientGoalHandle<Distance>;

  explicit ActionsQuizClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("actions_quiz_client_node", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<Distance>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "distance_as");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&ActionsQuizClient::send_goal, this));
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
                   "Waiting for an Action Server to become available...\n");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = Distance::Goal();
    goal_msg.seconds = 20;

    RCLCPP_INFO(this->get_logger(), "Sending goal:\n    seconds: %d\n",
                goal_msg.seconds);

    auto send_goal_options = rclcpp_action::Client<Distance>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ActionsQuizClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&ActionsQuizClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&ActionsQuizClient::result_callback, this, _1);
    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Distance>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

  void
  goal_response_callback(const GoalHandleDistance::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted with ID UUID: \n");
    }
  }

  void
  feedback_callback(GoalHandleDistance::SharedPtr,
                    const std::shared_ptr<const Distance::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback:\n    current_dist: %f\n",
                feedback->current_dist);
  }

  void result_callback(const GoalHandleDistance::WrappedResult &result) {
    this->goal_done_ = true;

    RCLCPP_INFO(this->get_logger(), "Result:\n    status: %d\ntotal_dist: %f\n",
                result.result->status, result.result->total_dist);

    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_ERROR(this->get_logger(), "Goal finished with status: SUCCEEDED");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    // for (auto res : result.result->status) {
    //   RCLCPP_INFO(this->get_logger(), res);
    // }
  }
}; // class ActionsQuizClient

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<ActionsQuizClient>();

  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}