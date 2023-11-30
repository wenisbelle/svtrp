#include <cstddef>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include <chrono>
#include <functional>
#include <cstdlib>
#include <thread>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_result_kind.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
    
    // Store all variables until I get everything into a .txt file

    geometry_msgs::msg::PoseStamped pose1;
    pose1.header.frame_id = "map";
    pose1.pose.position.x = 0;
    pose1.pose.position.y = 0;
    pose1.pose.position.z = 0.0;
    pose1.pose.orientation.x = 0.0;
    pose1.pose.orientation.y = 0.0;
    pose1.pose.orientation.z = 0.0;
    pose1.pose.orientation.w = 1.0;
    goal_poses.push_back(pose1);

    geometry_msgs::msg::PoseStamped pose2;
    pose2.header.frame_id = "map";
    pose2.pose.position.x = 2.20;
    pose2.pose.position.y = 2.86;
    pose2.pose.position.z = 0.0;
    pose2.pose.orientation.x = 0.0;
    pose2.pose.orientation.y = 0.0;
    pose2.pose.orientation.z = 0.7323598743914582;
    pose2.pose.orientation.w = 0.6809177735830718;
    goal_poses.push_back(pose2);

    
    geometry_msgs::msg::PoseStamped pose3;
    pose3.header.frame_id = "map";
    pose3.pose.position.x = 2.93;
    pose3.pose.position.y = 1.00;
    pose3.pose.position.z = 0.0;
    pose3.pose.orientation.x = 0.0;
    pose3.pose.orientation.y = 0.0;
    pose3.pose.orientation.z = 0.0;
    pose3.pose.orientation.w = 1.0;
    goal_poses.push_back(pose3);

    geometry_msgs::msg::PoseStamped pose4;
    pose4.header.frame_id = "map";
    pose4.pose.position.x = -2.55;
    pose4.pose.position.y = -3.63;
    pose4.pose.position.z = 0.0;
    pose4.pose.orientation.x = 0.0;
    pose4.pose.orientation.y = 0.0;
    pose4.pose.orientation.z = 0.12364745845764359;
    pose4.pose.orientation.w = 0.9923262094779949;
    goal_poses.push_back(pose4);

    geometry_msgs::msg::PoseStamped pose5;
    pose5.header.frame_id = "map";
    pose5.pose.position.x = -9.82;
    pose5.pose.position.y = 2.23;
    pose5.pose.position.z = 0.0;
    pose5.pose.orientation.x = 0.0;
    pose5.pose.orientation.y = 0.0;
    pose5.pose.orientation.z = 0.8079094295960842;
    pose5.pose.orientation.w = 0.5893066719202573;
    goal_poses.push_back(pose5);

    geometry_msgs::msg::PoseStamped pose6;
    pose6.header.frame_id = "map";
    pose6.pose.position.x = -6.00;
    pose6.pose.position.y = 3.11;
    pose6.pose.position.z = 0.0;
    pose6.pose.orientation.x = 0.0;
    pose6.pose.orientation.y = 0.0;
    pose6.pose.orientation.z = 0.0;
    pose6.pose.orientation.w = 1.0;
    goal_poses.push_back(pose6);

    geometry_msgs::msg::PoseStamped pose7;
    pose7.header.frame_id = "map";
    pose7.pose.position.x = -7.32;
    pose7.pose.position.y = -9.0;
    pose7.pose.position.z = 0.0;
    pose7.pose.orientation.x = 0.0;
    pose7.pose.orientation.y = 0.0;
    pose7.pose.orientation.z = -0.5422258328125239;
    pose7.pose.orientation.w = 0.8402327928799047;
    goal_poses.push_back(pose7);

    geometry_msgs::msg::PoseStamped pose8;
    pose8.header.frame_id = "map";
    pose8.pose.position.x = 2.60;
    pose8.pose.position.y = -1.0;
    pose8.pose.position.z = 0.0;
    pose8.pose.orientation.x = 0.0;
    pose8.pose.orientation.y = 0.0;
    pose8.pose.orientation.z = 0;
    pose8.pose.orientation.w = 1.0;
    goal_poses.push_back(pose8);

    geometry_msgs::msg::PoseStamped pose9;
    pose9.header.frame_id = "map";
    pose9.pose.position.x = 6.30;
    pose9.pose.position.y = -2.0;
    pose9.pose.position.z = 0.0;
    pose9.pose.orientation.x = 0.0;
    pose9.pose.orientation.y = 0.0;
    pose9.pose.orientation.z = -0.9140712174271898;
    pose9.pose.orientation.w = 0.4055537072585765;
    goal_poses.push_back(pose9);

    geometry_msgs::msg::PoseStamped pose10;
    pose10.header.frame_id = "map";
    pose10.pose.position.x = 10.5;
    pose10.pose.position.y = 1.86;
    pose10.pose.position.z = 0.0;
    pose10.pose.orientation.x = 0.0;
    pose10.pose.orientation.y = 0.0;
    pose10.pose.orientation.z = 0.0;
    pose10.pose.orientation.w = 1.0;
    goal_poses.push_back(pose10);

    geometry_msgs::msg::PoseStamped pose11;
    pose11.header.frame_id = "map";
    pose11.pose.position.x = 12.3;
    pose11.pose.position.y = 5.30;
    pose11.pose.position.z = 0.0;
    pose11.pose.orientation.x = 0.0;
    pose11.pose.orientation.y = 0.0;
    pose11.pose.orientation.z = 0.6634877515644353;
    pose11.pose.orientation.w = 0.7481871447198022;
    goal_poses.push_back(pose11);

    geometry_msgs::msg::PoseStamped pose12;
    pose12.header.frame_id = "map";
    pose12.pose.position.x = 13.3;
    pose12.pose.position.y = 9.40;
    pose12.pose.position.z = 0.0;
    pose12.pose.orientation.x = 0.0;
    pose12.pose.orientation.y = 0.0;
    pose12.pose.orientation.z = 0.6484704937485642;
    pose12.pose.orientation.w = 0.761239790563718;
    goal_poses.push_back(pose12);

    geometry_msgs::msg::PoseStamped pose13;
    pose13.header.frame_id = "map";
    pose13.pose.position.x = 13.3;
    pose13.pose.position.y = 9.40;
    pose13.pose.position.z = 0.0;
    pose13.pose.orientation.x = 0.0;
    pose13.pose.orientation.y = 0.0;
    pose13.pose.orientation.z = 0.9530570515006865;
    pose13.pose.orientation.w = 0.3027907802176576;
    goal_poses.push_back(pose13);

    geometry_msgs::msg::PoseStamped pose14;
    pose14.header.frame_id = "map";
    pose14.pose.position.x = 13.3;
    pose14.pose.position.y = 9.40;
    pose14.pose.position.z = 0.0;
    pose14.pose.orientation.x = 0.0;
    pose14.pose.orientation.y = 0.0;
    pose14.pose.orientation.z = 0.9530570515006865;
    pose14.pose.orientation.w = 0.3027907802176576;
    goal_poses.push_back(pose14);

    geometry_msgs::msg::PoseStamped pose15;
    pose15.header.frame_id = "map";
    pose15.pose.position.x = 12.6;
    pose15.pose.position.y = -0.8;
    pose15.pose.position.z = 0.0;
    pose15.pose.orientation.x = 0.0;
    pose15.pose.orientation.y = 0.0;
    pose15.pose.orientation.z = -0.7071076667670175;
    pose15.pose.orientation.w = 0.7071058956049684;
    goal_poses.push_back(pose15);

    geometry_msgs::msg::PoseStamped pose16;
    pose16.header.frame_id = "map";
    pose16.pose.position.x = 16.9;
    pose16.pose.position.y = 1.64;
    pose16.pose.position.z = 0.0;
    pose16.pose.orientation.x = 0.0;
    pose16.pose.orientation.y = 0.0;
    pose16.pose.orientation.z = -0.3366044341216231;
    pose16.pose.orientation.w = 0.9416461410368875;
    goal_poses.push_back(pose16);

    geometry_msgs::msg::PoseStamped pose17;
    pose17.header.frame_id = "map";
    pose17.pose.position.x = 21.2;
    pose17.pose.position.y = -2.20;
    pose17.pose.position.z = 0.0;
    pose17.pose.orientation.x = 0.0;
    pose17.pose.orientation.y = 0.0;
    pose17.pose.orientation.z = 0;
    pose17.pose.orientation.w = 1.0;
    goal_poses.push_back(pose17);

    geometry_msgs::msg::PoseStamped pose18;
    pose18.header.frame_id = "map";
    pose18.pose.position.x = 21.2;
    pose18.pose.position.y = -2.20;
    pose18.pose.position.z = 0.0;
    pose18.pose.orientation.x = 0.0;
    pose18.pose.orientation.y = 0.0;
    pose18.pose.orientation.z = 0;
    pose18.pose.orientation.w = 1.0;
    goal_poses.push_back(pose18);

  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    //criar funcao pra isso
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the range for the random floating-point number (0 to 17)
    std::uniform_int_distribution<> distribution(0, 17);
    random_number = distribution(gen);


    RCLCPP_INFO(this->get_logger(),
                  "The random namber is: %d", random_number);

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

    goal_msg.pose = goal_poses[random_number];


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
    
    RCLCPP_INFO(this->get_logger(), "Goal sent thesse coordinates: X: %f e Y: %f", goal_msg.pose.pose.position.x,
                                                                                   goal_msg.pose.pose.position.y);
  }

private:
  rclcpp_action::Client<GoToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_publisher_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  bool goal_done_;
  int random_number;
  std::vector<geometry_msgs::msg::PoseStamped> goal_poses;
  

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
      std::this_thread::sleep_for(3000ms);  

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

   
    send_goal();

  }

  void broadcast_timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = "actual_goal";
    t.transform.translation.x = goal_poses[random_number].pose.position.x;
    t.transform.translation.y = goal_poses[random_number].pose.position.y;    
    t.transform.translation.z = 0;
    t.transform.rotation.x = 0;    
    t.transform.rotation.y = 0;
    t.transform.rotation.z = goal_poses[random_number].pose.orientation.z;
    t.transform.rotation.w = goal_poses[random_number].pose.orientation.w;

    tf_broadcaster_publisher_->sendTransform(t);
  }

  /*// Function to generate and push pose into the vector
  void pushPose(std::vector<geometry_msgs::msg::PoseStamped>& poses, const std::string& frame_id,
                double pos_x, double pos_y, double pos_z,
                double orient_x, double orient_y, double orient_z, double orient_w) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.pose.position.x = pos_x;
      pose.pose.position.y = pos_y;
      pose.pose.position.z = pos_z;
      pose.pose.orientation.x = orient_x;
      pose.pose.orientation.y = orient_y;
      pose.pose.orientation.z = orient_z;
      pose.pose.orientation.w = orient_w;
      poses.push_back(pose);
}*/


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