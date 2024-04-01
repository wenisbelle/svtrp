#include <cstddef>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <sstream>
#include <fstream>

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
    
    std::string file_path = "/home/wenis/Portifolio/ROS/SVTRP/ros2/src/nav2_features/src/poses.txt";
    std::ifstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "Unable to open file: " << file_path << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float value;
        std::vector<float> elements;

        // Read values separated by spaces and store them in a vector
        while (iss >> value) {
            elements.push_back(value);
        }
        
        poses.push_back(elements); // Store elements for each line
    }

    file.close();
    std::cout << "All the poses" << std::endl;

    for (const auto& lineElements : poses) {
        for (const auto& element : lineElements) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }

  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    //criar funcao pra isso
    std::random_device rd;
    std::mt19937 gen(rd());

    // Define the range for the random floating-point number (0 to 15)
    std::uniform_int_distribution<> distribution(0, 15);
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

    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = poses[random_number][0];
    goal_msg.pose.pose.position.y = poses[random_number][1];
    goal_msg.pose.pose.position.z = 0;
    goal_msg.pose.pose.orientation.x = 0;
    goal_msg.pose.pose.orientation.y = 0;
    goal_msg.pose.pose.orientation.z = poses[random_number][2];
    goal_msg.pose.pose.orientation.w = poses[random_number][3];
     
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
  std::vector<std::vector<float>> poses;
  

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
    t.transform.translation.x = poses[random_number][0];
    t.transform.translation.y = poses[random_number][1];    
    t.transform.translation.z = 0;
    t.transform.rotation.x = 0;    
    t.transform.rotation.y = 0;
    t.transform.rotation.z = poses[random_number][2];
    t.transform.rotation.w = poses[random_number][3];;

    tf_broadcaster_publisher_->sendTransform(t);
  }


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