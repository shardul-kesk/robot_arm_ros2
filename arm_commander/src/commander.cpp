#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJT>;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("commander");
  auto client = rclcpp_action::create_client<FollowJT>(
    node, "/arm_controller/follow_joint_trajectory");

  if (!client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available");
    return 1;
  }

  // The goal: which joints, and a list of poses to pass through
  FollowJT::Goal goal;
  goal.trajectory.joint_names = {
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"};

  // Each pose is 5 joint angles (radians) plus the time (seconds) to reach it
  struct Pose { std::vector<double> angles; double seconds; };
  std::vector<Pose> poses = {
    {{0.0, 0.0, 0.0, 0.0, 0.0}, 2.0},
    {{0.6, 0.4, 0.4, 0.0, 0.0}, 5.0},
    {{0.6, 0.8, 0.8, 0.0, 0.0}, 8.0},
    {{-0.6, 0.4, 0.4, 0.0, 0.0}, 12.0},
    {{0.0, 0.0, 0.0, 0.0, 0.0}, 15.0},
  };
  for (const auto & p : poses) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = p.angles;
    point.time_from_start = rclcpp::Duration::from_seconds(p.seconds);
    goal.trajectory.points.push_back(point);
  }

  // Print progress while the arm moves
  auto options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
  options.feedback_callback =
    [node](GoalHandle::SharedPtr, const std::shared_ptr<const FollowJT::Feedback> fb) {
      if (!fb->actual.positions.empty()) {
        RCLCPP_INFO(node->get_logger(), "Shoulder pan at %.2f rad", fb->actual.positions[0]);
      }
    };

  // Send the goal and wait for the controller to accept it
  auto goal_future = client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    return 1;
  }
  auto handle = goal_future.get();
  if (!handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
    return 1;
  }

  // Wait for the arm to finish and report the result
  auto result_future = client->async_get_result(handle);
  rclcpp::spin_until_future_complete(node, result_future);
  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node->get_logger(), "Sequence finished");
  } else {
    RCLCPP_WARN(node->get_logger(), "Sequence did not succeed");
  }

  rclcpp::shutdown();
  return 0;
}
