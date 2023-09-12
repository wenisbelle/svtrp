#include "custom_messages/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/parameter_service.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <unistd.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using namespace std;
using FindWall = custom_messages::srv::FindWall;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("find_wall_server_node") {

    callback_service_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_subscriber_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_sub;
    options_sub.callback_group = callback_subscriber_group_;

    srv_ = create_service<FindWall>(
        "findwall", std::bind(&ServerNode::findwall_callback, this, _1, _2),
        rmw_qos_profile_services_default, callback_service_group_);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("robot_base_controller/cmd_vel_unstamped", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&ServerNode::topic_callback, this, std::placeholders::_1),
        options_sub);
  }

private:
  void MoveForward() {
    vx = 0.1;
    vz = 0.0;
    std::cout << "Move Forward" << std::endl;
  }

  void TurnClockwise() {
    vx = 0;
    vz = -0.2;
    std::cout << "Turn clockwise" << std::endl;
  }

  void TurnCounterClockwise() {
    vx = 0;
    vz = 0.2;
    std::cout << "Turn counter clockwise" << std::endl;
  }

  void PubVelocity() {
    this->twist_msg.linear.x = vx;
    this->twist_msg.angular.z = vz;
    this->publisher_->publish(this->twist_msg);
  }

  void Stop() {
    vx = 0.0;
    vz = 0.0;
  }

  void findwall_callback(const std::shared_ptr<FindWall::Request> request,
                         const std::shared_ptr<FindWall::Response> response) {

    while (final_position != true) {


      if (Foward > min_distance + 0.2 && min_distance >= 1.00) {
        TurnClockwise();

      } else if (Foward <= min_distance + 0.25 && min_distance >= 1.00) {
        MoveForward();
      } else if (Right > min_distance + 0.2 && min_distance <= 1.00) {
        TurnCounterClockwise();
      } else if (Right <= min_distance + 0.2 && min_distance <= 1.00) {
        Stop();
        PubVelocity();
        response->wallfound = true;
        final_position = true;
        RCLCPP_INFO(this->get_logger(), "He found the wall");
        std::this_thread::sleep_for(5000ms);
        // rclcpp::shutdown();
      } else {
        TurnCounterClockwise();
      }

      std::cout<< "Forward: " << Foward << std::endl;
      std::cout<< "Right: " << Right << std::endl; 
      std::cout<< "Min Distance: " << min_distance << std::endl;

      // Publish the right velocity
      PubVelocity();
      std::this_thread::sleep_for(1500ms);

      // Stop for a while
      Stop();
      PubVelocity();
      // RCLCPP_INFO(this->get_logger(), "Min dist %f", min_distance);
      std::this_thread::sleep_for(500ms);
    }
  }

  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    std::this_thread::sleep_for(50ms);
    Foward = msg->ranges[100];
    Right = msg->ranges[50];

    min_distance = 100;
    // finding the minimal value
    for (int i = 0; i < 200; i++) {
      if ((msg->ranges[i] <= min_distance) &&
          (msg->ranges[i] >= msg->range_min) &&
          (msg->ranges[i] <= msg->range_max)) {
        min_distance = msg->ranges[i];
        position = i;
      }
    }
  }

  float Foward;
  float Right;
  float min_distance;
  float vx;
  float vz;
  int position;
  bool final_position = false;
  rclcpp::Service<FindWall>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  geometry_msgs::msg::Twist twist_msg;
  rclcpp::CallbackGroup::SharedPtr callback_service_group_;
  rclcpp::CallbackGroup::SharedPtr callback_subscriber_group_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<ServerNode> server_node = std::make_shared<ServerNode>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}