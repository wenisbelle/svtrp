#include <cstddef>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>
#include <random>

#include <chrono>
#include <functional>
#include <cstdlib>
#include <thread>

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_result_kind.hpp"
#include "rclcpp_action/rclcpp_action.hpp"




using namespace std::chrono_literals;


class MyActionClient : public rclcpp::Node {
public:
  using GoThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleGoThroughPoses = rclcpp_action::ClientGoalHandle<GoThroughPoses>;

  explicit MyActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("go_through_poses_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<GoThroughPoses>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "navigate_through_poses");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&MyActionClient::send_goal, this));

  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    
    // First Pose
    geometry_msgs::msg::PoseStamped pose1;
    pose1.header.frame_id = "map";
    pose1.pose.position.x = -1.0;
    pose1.pose.position.y = 1.0;
    pose1.pose.position.z = 0.0;
    pose1.pose.orientation.x = 0.0;
    pose1.pose.orientation.y = 0.0;
    pose1.pose.orientation.z = 0.0;
    pose1.pose.orientation.w = 1.0;

    // Second Pose
    geometry_msgs::msg::PoseStamped pose2;
    pose2.header.frame_id = "map";
    pose2.pose.position.x = -3.0;
    pose2.pose.position.y = 0.0;
    pose2.pose.position.z = 0.0;
    pose2.pose.orientation.x = 0.0;
    pose2.pose.orientation.y = 0.0;
    pose2.pose.orientation.z = 0.0;
    pose2.pose.orientation.w = 1.0;

    // Third Pose
    geometry_msgs::msg::PoseStamped pose3;
    pose3.header.frame_id = "map";
    pose3.pose.position.x = -3.0;
    pose3.pose.position.y = 0.5;
    pose3.pose.position.z = 0.0;
    pose3.pose.orientation.x = 0.0;
    pose3.pose.orientation.y = 0.0;
    pose3.pose.orientation.z = 0.0;
    pose3.pose.orientation.w = 1.0;

    // Third Pose
    geometry_msgs::msg::PoseStamped pose4;
    pose4.header.frame_id = "map";
    pose4.pose.position.x = -3.0;
    pose4.pose.position.y = 1.0;
    pose4.pose.position.z = 0.0;
    pose4.pose.orientation.x = 0.0;
    pose4.pose.orientation.y = 0.0;
    pose4.pose.orientation.z = 0.0;
    pose4.pose.orientation.w = 1.0;

    auto goal_msg = GoThroughPoses::Goal();
    goal_msg.poses.push_back(pose1);
    goal_msg.poses.push_back(pose2);
    goal_msg.poses.push_back(pose3);
    goal_msg.poses.push_back(pose4);


    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<GoThroughPoses>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&MyActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&MyActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&MyActionClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    
    RCLCPP_INFO(this->get_logger(), "Goal sent.");
  }

private:
  rclcpp_action::Client<GoThroughPoses>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool goal_done_;

  

  void goal_response_callback(const GoalHandleGoThroughPoses::SharedPtr &goal_handle) {

    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleGoThroughPoses::SharedPtr,
      const std::shared_ptr<const GoThroughPoses::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback received. X: %f e Y: %f",
                feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
    
    // sleep one second
      std::this_thread::sleep_for(1000ms);  

  }

  void result_callback(const GoalHandleGoThroughPoses::WrappedResult &result) {
    this->goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

  }

  // rclcpp::shutdown();
}; // class MyActionClient

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}