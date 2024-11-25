#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class ParamExercise : public rclcpp::Node {
public:
  ParamExercise()
      : Node("param_exercise_node"), get_laser_data_parameter_(false),
        get_odom_data_parameter_(false), distance_front(0.0),
        position({0.0, 0.0}) {
    // Define ParameterDescriptors
    auto param_desc_laser = rcl_interfaces::msg::ParameterDescriptor{};
    auto param_desc_odom = rcl_interfaces::msg::ParameterDescriptor{};

    param_desc_laser.description =
        "Flag: Display value of the laser right in front of the robot";
    param_desc_odom.description =
        "Flag: Display value of the [X, Y] position of the robot";

    // Declare parameters
    this->declare_parameter<bool>("get_laser_data", false, param_desc_laser);
    this->declare_parameter<bool>("get_odom_data", false, param_desc_odom);

    // Create a timer
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&ParamExercise::timer_callback, this));

    // Subscribe to LaserScan topic
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ParamExercise::laser_scan_callback, this,
                  std::placeholders::_1));

    // Subscribe to Odometry topic
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ParamExercise::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Parameter Exercise Node Ready");
  }

private:
  bool get_laser_data_parameter_, get_odom_data_parameter_;
  double distance_front;
  std::pair<double, double> position;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  void timer_callback() {
    // Get the parameter values
    this->get_parameter("get_laser_data", get_laser_data_parameter_);
    this->get_parameter("get_odom_data", get_odom_data_parameter_);

    if (get_laser_data_parameter_) {
      RCLCPP_INFO(this->get_logger(), "Distance at front: %.2f m",
                  distance_front);
    }

    if (get_odom_data_parameter_) {
      RCLCPP_INFO(this->get_logger(), "Position of the robot: [%.2f, %.2f]",
                  position.first, position.second);
    }
  }

  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!msg->ranges.empty()) {
      size_t mid_index = msg->ranges.size() / 2;
      distance_front = msg->ranges[mid_index];
    } else {
      RCLCPP_WARN(this->get_logger(), "LaserScan message has empty ranges");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position.first = msg->pose.pose.position.x;
    position.second = msg->pose.pose.position.y;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamExercise>());
  rclcpp::shutdown();
  return 0;
}
