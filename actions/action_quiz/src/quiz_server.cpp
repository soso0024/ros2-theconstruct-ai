#include <functional>
#include <memory>
#include <thread>

#include "action_quiz_msg/action/distance.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_action/types.hpp"

class QuizServer : public rclcpp::Node {
public:
  using Distance = action_quiz_msg::action::Distance;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Distance>;

  explicit QuizServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("quiz_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Distance>(
        this, "distance_as", std::bind(&QuizServer::handle_goal, this, _1, _2),
        std::bind(&QuizServer::handle_cancel, this, _1),
        std::bind(&QuizServer::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("total_distance", 10);
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Distance::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
                goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&QuizServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto &message = feedback->current_dist;
    message = 1;
    auto result = std::make_shared<Distance::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); i++) {
      if (goal_handle->is_canceling()) {
        result->status = message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      message = 1;
      move.linear.x = 0.5;
      publisher_->publish(move);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->status = "Finished action server. Robot moved during 20 seconds";
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<QuizServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

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

user:~/ros2_ws$ ros2 action list
/move_robot_as
user:~/ros2_ws$ ros2 action info /move_robot_as
Action: /move_robot_as
Action clients: 0
Action servers: 1
    /t3_action_server
user:~/ros2_ws$ ros2 action info /move_robot_as -t
Action: /move_robot_as
Action clients: 0
Action servers: 1
    /t3_action_server [t3_action_msg/action/Move]
*/