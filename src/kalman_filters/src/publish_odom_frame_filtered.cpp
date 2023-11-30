#include <chrono>
#include <functional>
#include <memory>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <thread>


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/odometry.hpp>


using namespace std::chrono_literals;


class OdomFilteredBroadcaster : public rclcpp::Node
{
public:
  OdomFilteredBroadcaster()
  : Node("odom_filtered_frame_broadcaster"){
    tf_broadcaster_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    publisher_timer_ = this->create_wall_timer(
      50ms, std::bind(&OdomFilteredBroadcaster::broadcast_timer_callback, this));

    odom_filtered_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10,
        std::bind(&OdomFilteredBroadcaster::subscription_callback, this, std::placeholders::_1));
  }

private:
  void broadcast_timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom_filtered";
    t.child_frame_id = "base_link";
    t.transform.translation.x = std::move(odom_filtered_msg_.pose.pose.position.x);
    t.transform.translation.y = std::move(odom_filtered_msg_.pose.pose.position.y);    
    t.transform.translation.z = std::move(odom_filtered_msg_.pose.pose.position.z);
    t.transform.rotation.x = std::move(odom_filtered_msg_.pose.pose.orientation.x);    
    t.transform.rotation.y = std::move(odom_filtered_msg_.pose.pose.orientation.y);
    t.transform.rotation.z = std::move(odom_filtered_msg_.pose.pose.orientation.z);
    t.transform.rotation.w = std::move(odom_filtered_msg_.pose.pose.orientation.w);

    tf_broadcaster_publisher_->sendTransform(t);
  }

void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "I heard at the time: '%d'", msg->header.stamp.nanosec);
    odom_filtered_msg_ = *msg;
    std::this_thread::sleep_for(10ms);    
}

rclcpp::TimerBase::SharedPtr publisher_timer_;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_publisher_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_subscription_;
nav_msgs::msg::Odometry odom_filtered_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFilteredBroadcaster>());
  rclcpp::shutdown();
  return 0;
}