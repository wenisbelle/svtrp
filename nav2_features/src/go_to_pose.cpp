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

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_result_kind.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"



using namespace std::chrono_literals;


class MyActionClient : public rclcpp::Node {
public:
  using GoToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

  explicit MyActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("go2pose_client", node_options), goal_done_(false) {
    this->client_ptr_ = rclcpp_action::create_client<GoToPose>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "navigate_to_pose");

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&MyActionClient::send_goal, this));

    tf_broadcaster_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    publisher_timer_ = this->create_wall_timer(
      50ms, std::bind(&MyActionClient::broadcast_timer_callback, this));
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

    auto goal_msg = GoToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = x_pos;
    goal_msg.pose.pose.position.y = y_pos;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<GoToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        std::bind(&MyActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
        std::bind(&MyActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
        std::bind(&MyActionClient::result_callback, this, _1);

    auto goal_handle_future =
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    
    RCLCPP_INFO(this->get_logger(), "Goal sent thesse coordinates: X: %f e Y: %f", x_pos, y_pos);
  }

private:
  rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_publisher_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  bool goal_done_;
  float x_pos = -1.0;
  float y_pos = 1.0;
  

  void goal_response_callback(const GoalHandleGoToPose::SharedPtr &goal_handle) {

    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleGoToPose::SharedPtr,
      const std::shared_ptr<const GoToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback received. X: %f e Y: %f",
                feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y);
    
    // sleep one second
      std::this_thread::sleep_for(1000ms);  

  }

  void result_callback(const GoalHandleGoToPose::WrappedResult &result) {
    //this->goal_done_ = true;
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

    std::random_device rd;
    std::mt19937 gen_x(rd());
    std::mt19937 gen_y(rd());

    // Define the range for the random floating-point number (-10 to 10)
    std::uniform_real_distribution<float> floatDistribution(-5.0f, 5.0f);

    x_pos = floatDistribution(gen_x);
    y_pos = floatDistribution(gen_y);
    send_goal();

  }

  void broadcast_timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = "actual_goal";
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;    
    t.transform.translation.z = 0;
    t.transform.rotation.x = 0;    
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 1;

    tf_broadcaster_publisher_->sendTransform(t);
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