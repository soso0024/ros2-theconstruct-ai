#include <functional>
#include <math.h>
#include <memory>
#include <thread>

#include "actions_quiz_msg/action/distance.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
#include "std_msgs/msg/detail/float64__struct.hpp"
#include "std_msgs/msg/float64.hpp"

class QuizServer : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Distance>;

  explicit QuizServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("quiz_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Distance>(
        this, "distance_as", std::bind(&QuizServer::handle_goal, this, _1, _2),
        std::bind(&QuizServer::handle_cancel, this, _1),
        std::bind(&QuizServer::handle_accepted, this, _1));

    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    publisher_dis_ =
        this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);

    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&QuizServer::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_dis_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;

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

  double last_x_, last_y_;
  bool initialized_ = false;
  double total_distance_ = 0.0;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    if (initialized_) {
      double dist_increment =
          sqrt(pow(current_x - last_x_, 2) + pow(current_y - last_y_, 2));
      total_distance_ += dist_increment;
    } else {
      initialized_ = true;
    }
    last_x_ = current_x;
    last_y_ = current_y;
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto result = std::make_shared<Distance::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); i++) {
      if (goal_handle->is_canceling()) {
        result->status = false;
        result->total_dist = total_distance_;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      move.linear.x = 0.5;
      publisher_vel_->publish(move);

      // Have to Change
      //   total_distance_ += move.linear.x;
      //   feedback->current_dist = move.linear.x;

      std_msgs::msg::Float64 distance_msg;
      distance_msg.data = total_distance_;
      publisher_dis_->publish(distance_msg);
      feedback->current_dist = total_distance_;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    if (rclcpp::ok()) {
      result->status = true;
      result->total_dist = total_distance_;
      move.linear.x = 0.0;
      publisher_vel_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

/*
< To Do >

ロボットの初期座標を０基準として，そのからの時間経過での移動距離を表示するようにする．
現状は下記のような結果となっている

user:~/ros2_ws$ ros2 action send_goal -f /distance_as
actions_quiz_msg/action/Distance "{seconds: 3}" Waiting for an action server to
become available... Sending goal: seconds: 3

Goal accepted with ID: 9ae811527158423da6858d82bd1c178f

Feedback:
    current_dist: 9.27453502506758

Feedback:
    current_dist: 9.772634628826907

Feedback:
    current_dist: 10.279983609530971

Result:
    status: true
total_dist: 10.786844103707107

Goal finished with status: SUCCEEDED
*/

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
<Action Run Command>
ros2 action send_goal -f /distance_as actions_quiz_msg/action/Distance
"{seconds: 20}"



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


user:~/ros2_ws$ ros2 interface show std_msgs/msg/Float64
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in
example_msgs.

float64 data


ros2 topic echo /odom
header:
  stamp:
    sec: 9264
    nanosec: 577000000
  frame_id: odom
child_frame_id: base_footprint
pose:
  pose:
    position:
      x: 3.1116432711447706e-06
      y: -8.13500430466293e-09
      z: -0.0007969010028527632
    orientation:
      x: 1.654776810878313e-09
      y: 0.0007945752173001615
      z: -1.5797427127425023e-09
      w: 0.9999996843250624
  covariance:
  - 1.0e-05
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0e-05
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.001
twist:
  twist:
    linear:
      x: 3.9466436353717e-05
      y: -3.939381090724338e-07
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: -1.1199097272844502e-07
  covariance:
  - 1.0e-05
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1.0e-05
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 1000000000000.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.001
---
*/