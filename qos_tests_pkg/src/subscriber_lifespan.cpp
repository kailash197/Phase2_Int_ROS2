#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <math.h>

using std::placeholders::_1;

class SubscriberQos : public rclcpp::Node {
public:
  SubscriberQos(int &argc, char **argv) : Node("subscriber_qos_obj") {

    lifespan = std::stof(argv[2]);
    lifespan_ms = lifespan * 1000;
    rclcpp::QoS qos_profile_subscriber(1);
    qos_profile_subscriber.lifespan(std::chrono::milliseconds(lifespan_ms));

    qos_profile_subscriber.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile_subscriber.durability(
        RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.event_callbacks.incompatible_qos_callback =
        std::bind(&SubscriberQos::incompatible_qos_info_callback, this, _1);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "qos_test", qos_profile_subscriber,
        std::bind(&SubscriberQos::listener_callback, this, _1),
        subscription_options);
  }

private:
  void incompatible_qos_info_callback(
      rclcpp::QOSRequestedIncompatibleQoSInfo &event) {
    RCLCPP_ERROR(this->get_logger(),
                 "SUBSCRIBER::: INCOMPATIBLE QoS Triggered!");
    RCLCPP_INFO(
        this->get_logger(),
        "Requested incompatible qos - total %d delta %d last_policy_kind: %d",
        event.total_count, event.total_count_change, event.last_policy_kind);
  }

  void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
    // The data has the format Seconds,NanoSeconds

    raw_data = msg->data;
    RCLCPP_INFO(this->get_logger(), "Data Received = %s seconds",
                raw_data.c_str());
    delimiter = ",";
    last = 0;
    next = 0;
    while ((next = raw_data.find(delimiter, last)) != std::string::npos) {
      seconds = std::stod(raw_data.substr(last, next - last));
      last = next + 1;
    }
    nanoseconds = std::stod(raw_data.substr(last));
    RCLCPP_INFO(this->get_logger(), "SPLIT = [%lf,%lf]", seconds, nanoseconds);
    RCLCPP_INFO(this->get_logger(), "seconds = %lf, nseconds = %lf", seconds,
                nanoseconds);

    total_seconds = seconds + pow(nanoseconds, -9);
    RCLCPP_INFO(this->get_logger(), "total_seconds = %lf", total_seconds);

    rclcpp::Time time_now_obj = this->now();
    total_current_time =
        time_now_obj.seconds() + pow((time_now_obj.nanoseconds()), -9);
    RCLCPP_INFO(this->get_logger(), "total_current_time = %lf",
                total_current_time);
    delta = total_current_time - total_seconds;
    RCLCPP_INFO(this->get_logger(), "Message Age = %lf seconds", delta);
    // Message Age will show the number of seconds elapsed between the start of
    // the publisher and the start of the subscriber.
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  float lifespan;
  int lifespan_ms;
  double total_seconds;
  double total_current_time;
  double seconds;
  double nanoseconds;
  size_t last;
  size_t next;
  double delta;
  std::string raw_data;
  std::string delimiter;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberQos>(argc, argv));
  rclcpp::shutdown();
  return 0;
}

/*
cd ~/ros2_ws/
colcon build --packages-select qos_tests_pkg
source install/setup.bash
ros2 run qos_tests_pkg publisher_lifespan_exe -lifespan 20.0

source ~/ros2_ws/install/setup.bash
ros2 run qos_tests_pkg subscriber_lifespan_exe -lifespan 1.0
[INFO] [1680131593.313753596] [subscriber_qos_obj]: Data Received =
1680131588.319890,1680131588319890176.000000 seconds [INFO]
[1680131593.313841995] [subscriber_qos_obj]: SPLIT =
[1680131588.319890,1680131588319890176.000000] [INFO] [1680131593.313881163]
[subscriber_qos_obj]: seconds = 1680131588.319890, nseconds =
1680131588319890176.000000 [INFO] [1680131593.313921224] [subscriber_qos_obj]:
total_seconds = 1680131588.319890 [INFO] [1680131593.313943456]
[subscriber_qos_obj]: total_current_time = 1680131593.313942 [INFO]
[1680131593.313964152] [subscriber_qos_obj]: Message Age = 4.994052 seconds



Too restrictive Publisher lifespan¶
To make this test, you must start the subscriber first; otherwise, you can not
measure the time correctly.

   Shell #1
ros2 run qos_tests_pkg subscriber_lifespan_exe -lifespan 1.0

   Shell #2
Launch a fast lifespan, but doable by your system


ros2 run qos_tests_pkg publisher_lifespan_exe -lifespan 0.00001
   Shell Output #2

As you can see, the message Age is 0.000424 seconds, so in theory, it SHOULD NOT
have worked. However, the subscriber was much faster than the time it took the
Callback to make the calculations, and log inside the Callback. It received the
message in 0.00001 seconds lifespan of the publisher.
[INFO] [1680132077.273447403] [subscriber_qos_obj]: Data Received =
1680132077.273251,1680132077273251072.000000 seconds [INFO]
[1680132077.273569591] [subscriber_qos_obj]: SPLIT =
[1680132077.273251,1680132077273251072.000000] [INFO] [1680132077.273608618]
[subscriber_qos_obj]: seconds = 1680132077.273251, nseconds =
1680132077273251072.000000 [INFO] [1680132077.273652540] [subscriber_qos_obj]:
total_seconds = 1680132077.273251 [INFO] [1680132077.273676718]
[subscriber_qos_obj]: total_current_time = 1680132077.273675 [INFO]
[1680132077.273697039] [subscriber_qos_obj]: Message Age = 0.000424 seconds

*/