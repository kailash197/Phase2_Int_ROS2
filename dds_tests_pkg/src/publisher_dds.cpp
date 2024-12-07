#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

class PublisherDDS : public rclcpp::Node {
public:
  PublisherDDS() : Node("publisher_dds_obj") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("dds_test", 1);
    timer_ = this->create_wall_timer(
        0ms, std::bind(&PublisherDDS::timer_callback, this));
    /* try making 0ms to overflow the system and force it to lose some messages
    The effects may vary, depending on the system you are executing this in:

    In an 8 core system, you can get losses of around 9 messages maximum.
    In a 12-core system, the loss is only 1 message.
    */
    msgs_id = 0;
  }

private:
  void timer_callback() {
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
    ++msgs_id;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int msgs_id;
  std::string time_str;
  std::string dds_msg_str;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherDDS>());
  rclcpp::shutdown();
  return 0;
}
/*
The publisher generates a unique ID for each of the messages to be sent and its
generation time. It then publishes this information.

Then the subscriber gets that message, and compares the message generation time
with the current time. That way, you know if a particular Middleware has more
overhead time than the other. On the other hand, it registers the messages ID,
and checks if that is the message ID it was hoping to get next. Because the
message IDs are sequential, it is easy to know with this information if you have
lost a message and how many.

*/