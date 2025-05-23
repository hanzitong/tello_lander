
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "controller.hpp"

using namespace std::chrono_literals;

class ControlInputPublisher : public rclcpp::Node
{
  public:
    ControlInputPublisher()
    : Node("control_input_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      subscriber_ = this->create_subscriber<>();
      timer_ = this->create_wall_timer(
      100ms, std::bind(&ControlInputPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        // auto message = std_msgs::msg::String();
        auto cmd_vel = geometry_msgs::msg::Twist();

        // message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", cmd_vel.data.c_str());
        publisher_->publish(cmd_vel);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlInputPublisher>());
  rclcpp::shutdown();
  return 0;
}

