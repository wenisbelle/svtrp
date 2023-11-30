#include <chrono>
#include <cmath>
#include <functional>
#include <math.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/detail/point32__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_messages/action/odom_record.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class MyActionServer : public rclcpp::Node {
public:
  using OdomRecord = custom_messages::action::OdomRecord;
  using GoalHandleOdomRecord = rclcpp_action::ServerGoalHandle<OdomRecord>;

  explicit MyActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("record_odom", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<OdomRecord>(
        this, "record", std::bind(&MyActionServer::handle_goal, this, _1, _2),
        std::bind(&MyActionServer::handle_cancel, this, _1),
        std::bind(&MyActionServer::handle_accepted, this, _1));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot_base_controller/odom", 10,
        std::bind(&MyActionServer::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  rclcpp_action::Server<OdomRecord>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  geometry_msgs::msg::Point32 actual_coords;
  std::vector<geometry_msgs::msg::Point32> coords;

  float distance;
  float delta;
  float DistanceFromStart;

  geometry_msgs::msg::Quaternion q;

  struct EulerAngles {
    float roll, pitch, yaw;
  };

  // Topic Callback function
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    q.x = msg->pose.pose.orientation.x;
    q.y = msg->pose.pose.orientation.y;
    q.z = msg->pose.pose.orientation.z;
    q.w = msg->pose.pose.orientation.w;
    actual_coords.x = msg->pose.pose.position.x;
    actual_coords.y = msg->pose.pose.position.y;
    actual_coords.z = msg->pose.pose.orientation.z;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const OdomRecord::Goal> goal) {
    // RCLCPP_INFO(this->get_logger(), "Received goal request: %d",
    // goal->request);
    (void)uuid;
    distance = 0;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  float calculateDistance(float &x1, float &y1, float &x2, float &y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  }

  EulerAngles ToEulerAngles(geometry_msgs::msg::Quaternion &q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
  }

  void execute(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<OdomRecord::Feedback>();
    auto &current_dist = feedback->current_total;
    current_dist = distance;

    auto result = std::make_shared<OdomRecord::Result>();
    // rclcpp::Rate loop_rate(1);
    bool lap = false;
    bool ExitFromStart = false;

    while (lap == false && rclcpp::ok()) {

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->list_of_odoms = coords;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      EulerAngles angles = ToEulerAngles(q);
      actual_coords.set__z(angles.yaw * 180 / 3.1415);

      coords.push_back(actual_coords);

      DistanceFromStart = calculateDistance(coords[0].x, coords[0].y,
                                            coords[coords.size() - 1].x,
                                            coords[coords.size() - 1].y);

      if (DistanceFromStart > 2.0) {
        ExitFromStart = true;
      }

      if (DistanceFromStart < 1.0 && ExitFromStart == true) {
        lap = true;
        RCLCPP_INFO(this->get_logger(), "The robot made the lap");
      }

      if (coords.size() == 1) {
        delta = 0;
      } else {
        delta = calculateDistance(
            coords[coords.size() - 2].x, coords[coords.size() - 2].y,
            coords[coords.size() - 1].x, coords[coords.size() - 1].y);
      }

      distance = distance + delta;
      current_dist = distance;
      goal_handle->publish_feedback(feedback);
      std::cout<< "Distance: " << DistanceFromStart << std::endl;

      // sleep one second
      std::this_thread::sleep_for(1000ms);
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->list_of_odoms = coords;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
}; // class MyActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}