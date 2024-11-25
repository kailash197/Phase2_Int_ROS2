#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class VelParam : public rclcpp::Node {
public:
  VelParam() : Node("param_vel_node") {
    // define a ParameterDescriptor
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    // declare parameter: name, default value, parameter descriptor
    param_desc.description = "Sets the velocity (in m/s) of the robot.";
    this->declare_parameter<std::double_t>("velocity", 0.0, param_desc);
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&VelParam::timer_callback, this));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
  void timer_callback() {
    // get value of parameter `velocity`
    this->get_parameter("velocity", vel_parameter_);
    RCLCPP_INFO(this->get_logger(), "Velocity parameter is: %f",
                vel_parameter_);
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = vel_parameter_;
    publisher_->publish(message);
  }

private:
  std::double_t vel_parameter_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelParam>());
  rclcpp::shutdown();
  return 0;
}