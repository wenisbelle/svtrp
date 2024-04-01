//#include "custom_interfaces/msg/battery.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;


class PublishBattery : public rclcpp::Node {
public:
  PublishBattery() : Node("battery_status") {
    publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("fake_battery", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PublishBattery::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = sensor_msgs::msg::BatteryState();
    double time_seconds = now().seconds();

    if (is_first_measurement) {
      this->initial_time = time_seconds;
      is_first_measurement = false;
    }

    message.voltage = 12;
    message.percentage = 1.0 - (time_seconds-initial_time)*0.05;

    if (message.percentage < 0.0)
    {
      message.percentage = 0.0;
    }
    
    RCLCPP_INFO(this->get_logger(), "Time elapsed: %f", message.percentage);
    publisher_->publish(message);
  }
  double initial_time;
  bool is_first_measurement = true;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishBattery>());
  rclcpp::shutdown();
  return 0;
}