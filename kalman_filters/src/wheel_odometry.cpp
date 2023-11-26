#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class MoveRobot : public rclcpp::Node {
public:
  MoveRobot() : Node("move_robot_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("robot_base_controller/cmd_vel_unstamped", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MoveRobot::timer_callback, this));

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&MoveRobot::topic_callback, this, std::placeholders::_1));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = vx;
    message.angular.z = vz;
    publisher_->publish(message);
  }

  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float Foward1 = msg->ranges[100]; // 360
    // float Foward2 = msg->ranges[460];
    // float Foward3 = msg->ranges[260];
    float Right = msg->ranges[50]; // 180
    //float Left = msg->ranges[150];  // 520
  

    if (Right > 2.0 && Foward1 > 2.5) {
      vz = -0.5;
      vx = 0.5;
    } else if (Right < 1.0 && Foward1 > 2.5) {
      vz = 0.5;
      vx = 0.5;
    } else if (Right < 2.0 && Right > 1.0 && Foward1 > 2.5) {
      vz = 0;
      vx = 0.5;
    } else if (Foward1 < 2.5) {
      vz = 1;
      vx = 0;
    }

    //RCLCPP_INFO(this->get_logger(),
    //            "Foward: '%f'. Right: '%f'. Left:'%f'. Vz '%f'", Foward1, Right,
    //            Left, vz);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  float vx;
  float vz;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobot>());
  rclcpp::shutdown();
  return 0;
}