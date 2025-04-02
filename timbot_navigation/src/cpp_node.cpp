#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/range.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      publisher2_ = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic_range", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);


        auto message2= sensor_msgs::msg::Range();
        message2.header.stamp = this->get_clock()->now();
        message2.header.frame_id = "ultrasonic_sensor";
        message2.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        message2.field_of_view = 0.2618; // ประมาณ 15 องศา
        message2.min_range = 0.02;
        message2.max_range = 4.0;
        message2.range = 1.0; // ตั้งค่าระยะทางที่ได้จากเซ็นเซอร์

        // เผยแพร่ข้อความ
        publisher2_->publish(message2);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher2_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}