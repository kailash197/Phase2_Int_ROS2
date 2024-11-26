#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PublisherQoS : public rclcpp::Node {
public:
  PublisherQoS(int &argc, char **argv) : Node("publisher_qos_obj") {

    deadline = std::stof(argv[2]);
    deadline_ms = deadline * 1000;
    rclcpp::QoS qos_profile_publisher(1);
    qos_profile_publisher.deadline(std::chrono::milliseconds(deadline_ms));

    rclcpp::PublisherOptions publisher_options;
    publisher_options.event_callbacks.incompatible_qos_callback =
        std::bind(&PublisherQoS::incompatible_qos_info_callback, this, _1);
    publisher_options.event_callbacks.deadline_callback =
        std::bind(&PublisherQoS::incompatible_deadline_callback, this);

    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/qos_test", qos_profile_publisher, publisher_options);

    timer_ = this->create_wall_timer(
        1000ms, std::bind(&PublisherQoS::timer_callback, this));

    msgs_id = 0;
    timer_period = 1.0;
    swap_state_time = 5.0;
    time_pause = 2.0;
    counter = 0;
  }

  void incompatible_deadline_callback() {
    RCLCPP_ERROR(this->get_logger(), "PUBLISHER:::  Deadline Triggered!");
  }

  void
  incompatible_qos_info_callback(rclcpp::QOSOfferedIncompatibleQoSInfo &event) {
    RCLCPP_ERROR(this->get_logger(),
                 "A subscriber is asking for an INCOMPATIBLE QoS Triggered!");
    RCLCPP_ERROR(
        this->get_logger(),
        "Offered incompatible qos - total %d delta %d last_policy_kind: %d",
        event.total_count, event.total_count_change, event.last_policy_kind);
  }

  void publish_one_message() {
    auto msg = std_msgs::msg::String();
    auto current_time = this->now();
    auto current_time_s = current_time.seconds();
    auto current_time_ns = current_time.nanoseconds();
    time_str =
        std::to_string(current_time_s) + "," + std::to_string(current_time_ns);
    dds_msg_str = std::to_string(msgs_id) + ":" + time_str;
    msg.data = dds_msg_str;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s ", msg.data.c_str());
  }

  void timer_callback() {
    if (counter > int(swap_state_time / timer_period)) {
      delta = 0.1;
      delta_ms = delta * 1000;
      range_steps = int(time_pause / delta);
      for (int i = 0; i < range_steps; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(delta_ms));
        RCLCPP_INFO(this->get_logger(), "Paused = %f / %f ", (i * delta),
                    time_pause);
      }
      counter = 0;
    } else {
      publish_one_message();
      ++counter;
      RCLCPP_INFO(this->get_logger(), "Counter = %d", counter);
    }
  }

private:
  float deadline;
  float timer_period;
  float swap_state_time;
  float time_pause;
  float delta;
  int counter;
  int range_steps;
  int delta_ms;
  int deadline_ms;
  int msgs_id;
  std::string time_str;
  std::string dds_msg_str;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherQoS>(argc, argv));
  rclcpp::shutdown();
  return 0;
}

/*
cd ~/ros2_ws/
colcon build --packages-select qos_tests_pkg
source install/setup.bash
ros2 run qos_tests_pkg publisher_deadline_exe -deadline 10.0

source ~/ros2_ws/install/setup.bash
ros2 run qos_tests_pkg subscriber_deadline_exe -deadline 10.0



You can see that the subscriber has no error, even when the publisher did not
publish anything for approximately 2 seconds. This is because the subscriber
had 10.0 seconds of the deadline, which is inside the margin.

And the publisher does not have any error either because it is publishing
something with a period lower than its 10.0 seconds deadline.
*/

/* Exercise: 7.2
 Exercise 7.2 -
Test different values of the deadlines to see the different behaviors:

Publisher fails to meet its deadline:

Here you have to set a deadline lower than the time it takes the publisher to
publish its message. Subscriber does not receive the publisher message on time:

Set the deadline for the subscriber lower than the publisher's time to publish
each message. If the subscriber deadline is small enough, This will trigger the
Callback event in the pause loop or even in the normal publish loop. The
subscribers and publishers have incompatible QoS values.

Here you have to have a look at the table of compatibilities.

1. Publisher fails to meet its deadline:
The publisher publishes a message every 1.0 seconds. This means that if you set
a deadline lower than 1.0 seconds, it will trigger the deadline_qos_clb:

ros2 run qos_tests_pkg publisher_deadline_exe -deadline 0.99

You can also make it fail in the pause phase:

ros2 run qos_tests_pkg publisher_deadline_exe -deadline 1.0

Note that the Callback for the deadline is executed in 2.0 seconds, which it
should not be because the deadline was set to 1.0 seconds. The reason is that
you are not using multithreading here, which causes you only have ONE thread by
default, and therefore, the Callback will only be executed after finishing the
timer_callback.


2. Subscriber does not receive the publisher message on time:
In this case, set a deadline higher than the normal publishing rate, but lower
than the time it spends paused.
   Shell #1
ros2 run qos_tests_pkg publisher_deadline_exe -deadline 1.2

The publisher is not running. The deadline is not triggered in the subscriber.
When you start the publisher, it only gives the Deadline Trigger in the
publisher and the subscriber when the publisher is paused. This is because the
publisher and subscriber have to wait more than 1.2/1.8 seconds, respectively,
to get a new publish message.



3. The subscriber and publisher have incompatible QoS values:
If you look at the table of compatibility, you can trigger the QoS
incompatibility by setting:

deadline subscriber < deadline of publisher
   Shell #1


ros2 run qos_tests_pkg publisher_deadline_exe -deadline 1.5
   Shell Output #1


[INFO] [1644945489.570006750] [publisher_qos_obj]: Published:
"std_msgs.msg.String(data='1644945489,552461406')" [ERROR]
[1644945489.570866117] [publisher_qos_obj]: PUBLISHER::: INCOMPATIBLE QoS
Triggered! [ERROR] [1644945489.571476981] [publisher_qos_obj]:
rmw_qos_policy_kind_t.RMW_QOS_POLICY_DEADLINE [ERROR] [1644945489.571942584]
[publisher_qos_obj]: ############################ [INFO] [1644945490.553480357]
[publisher_qos_obj]: Published:
"std_msgs.msg.String(data='1644945490,552765208')" The publisher does not stop
but gives the error.

   Shell #2


ros2 run qos_tests_pkg subscriber_deadline_exe -deadline 1.0
   Shell Output #2


deadline==Duration(nanoseconds=1000000000)
[ERROR] [1644945489.566753281] [subscriber_qos_obj]: SUBSCRIBER::: INCOMPATIBLE
QoS Triggered! [ERROR] [1644945489.567304443] [subscriber_qos_obj]:
rmw_qos_policy_kind_t.RMW_QOS_POLICY_DEADLINE [ERROR] [1644945489.567773866]
[subscriber_qos_obj]: ############################



*/