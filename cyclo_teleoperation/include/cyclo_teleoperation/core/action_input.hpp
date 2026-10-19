// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once
#include <unordered_map>
#include <functional>
#include <deque>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "cyclo_teleoperation/core/robot_teleoperation.hpp"

namespace cyclo_teleoperation
{
// Converts shared ROS action inputs into validated per-group references. No robot names,
// joint counts, QP, ROS services or controller implementations are known here.
class ActionInput
{
public:
  ActionInput(
    rclcpp::Node & node, RobotTeleoperation & robot,
    std::function<bool()> enabled, std::function<std::string()> reference_type,
    std::function<std::string()> anchor_frame);
  void clear();
  void clearGroups(ControlGroupMask groups);
  void observeFrames(const std_msgs::msg::Header & header);
  ControlGroupMask freshGroups() const;
  bool hasGripper(ControlGroupId group) const {return gripper_received_.at(group);}
  const GroupCartesianReferences & cartesianReferences() const {return poses_;}

private:
  bool validStamp(
    const builtin_interfaces::msg::Time & message, const rclcpp::Time & previous,
    rclcpp::Time & stamp) const;
  rclcpp::Node & node_;
  RobotTeleoperation & robot_;
  std::function<bool()> enabled_;
  std::function<std::string()> reference_type_;
  std::function<std::string()> anchor_frame_;
  struct FrameSnapshot
  {
    rclcpp::Time stamp;
    std::unordered_map<std::string, Eigen::Affine3d> poses;
  };
  std::deque<FrameSnapshot> frames_;
  std::string frame_;
  double timeout_;
  GroupCartesianReferences poses_;
  std::vector<bool> joint_received_, pose_received_, gripper_received_;
  std::vector<rclcpp::Time> joint_times_, pose_times_;
  std::vector<rclcpp::Time> joint_stamps_, pose_stamps_;
  rclcpp::Time reset_time_{0, 0, RCL_ROS_TIME};
  std::vector<rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr> joints_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_inputs_;
  std::vector<JointTrajectoryChannel> direct_channels_;
  std::vector<rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr>
  direct_inputs_;
};
}  // namespace cyclo_teleoperation
