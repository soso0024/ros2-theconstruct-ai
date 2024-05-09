#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;

class WallFollow : public rclcpp::Node {
public:
  WallFollow() : Node("wall_follw_project") {
    pub_ = this->create_publisher<Twist>("cmd_vel", 10);

    sub_ = this->create_subscription<LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&WallFollow::scan_callback, this, _1));

    timer_ = this->create_wall_timer(
        50ms, std::bind(&WallFollow::timer_callback, this));

    this->laser_left = 0.0;
    this->laser_right = 0.0;
    this->laser_forward = 0.0;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_;
  Twist twist_msg;
  float laser_left, laser_right, laser_forward;

  void scan_callback(const LaserScan::SharedPtr msg) {
    this->laser_left = msg->ranges[180];
    this->laser_right = msg->ranges[540];
    this->laser_forward = msg->ranges[360];

    // RCLCPP_INFO(this->get_logger(), "[LEFT] = '%f'", this->laser_left);
    // RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
    // RCLCPP_INFO(this->get_logger(), "[FORWARD] = '%f'", this->laser_forward);
  }

  void timer_callback() {
    if (this->laser_left <= 0.2) {
      this->twist_msg.angular.z = 0.3; // rotate direction of clock (Right): '+'
      this->twist_msg.linear.x = 0.1;
      RCLCPP_INFO(this->get_logger(), "Turn Right 1!");
    } else if (this->laser_right <= 0.2) {
      this->twist_msg.angular.z = -0.3;
      this->twist_msg.linear.x = 0.1;
      RCLCPP_INFO(this->get_logger(), "Turn Left 2!");
    } else if (this->laser_left >= 0.3) {
      this->twist_msg.angular.z = 0.3;
      this->twist_msg.linear.x = 0.1;
      RCLCPP_INFO(this->get_logger(), "Turn Right 3!");
    } else if (this->laser_right >= 0.3) {
      this->twist_msg.angular.z = -0.3;
      this->twist_msg.linear.x = 0.1;
      RCLCPP_INFO(this->get_logger(), "Turn Left 4!");
    } else if (this->laser_forward <= 0.5) {
      this->twist_msg.angular.z = 0.3;
      this->twist_msg.linear.x = 0.1;
      RCLCPP_INFO(this->get_logger(), "Turn Right 5!");
    } else {
      this->twist_msg.linear.x = 0.1;
      RCLCPP_INFO(this->get_logger(), "Move Straight");
    }

    pub_->publish(twist_msg);
  }
};

/*
To Do
ほぼずっとTurn Left 2が実行される原因を調べる
*/

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollow>());
  rclcpp::shutdown();
  return 0;
}

/*
user:~/ros2_ws$ ros2 topic list
/clock
/cmd_vel
/imu
/joint_states
/odom
/parameter_events
/performance_metrics
/robot_description
/rosout
/scan
/tf
/tf_static



user:~/ros2_ws$ ros2 topic info /scan
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 0



user:~/ros2_ws$ ros2 interface show sensor_msgs/msg/LaserScan
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header # timestamp in the header is the acquisition time of
                             # the first ray in the scan.
                             #
                             # in frame frame_id, angles are measured around
                             # the positive Z axis (counterclockwise, if Z is
up) # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your
scanner # is moving, this will be used in interpolating position # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                             # (Note: values < range_min or > range_max should
be discarded) float32[] intensities        # intensity data [device-specific
units].  If your # device does not provide intensities, please leave # the array
empty.
*/