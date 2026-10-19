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

#include "cyclo_teleoperation/robots/ai_worker/ai_worker_teleoperation.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace cyclo_teleoperation::robots::ai_worker
{
std::string AIWorkerTeleoperation::parameterName(const std::string & name) const
{
  return parameter_prefix_.empty() ? name : parameter_prefix_ + "." + name;
}

bool AIWorkerTeleoperation::configure(
  rclcpp::Node & node,
  const std::string & parameter_prefix,
  ControlInterface::RequestCallback request_callback)
{
  node_ = &node;
  parameter_prefix_ = parameter_prefix;

  auto declare_string = [this](const std::string & name, const std::string & value) {
      const auto full_name = parameterName(name);
      if (!node_->has_parameter(full_name)) {
        node_->declare_parameter(full_name, value);
      }
    };
  declare_string("follower_urdf_path", "");
  declare_string("follower_srdf_path", "");
  // Optional portable geometry hint for input devices; never instantiated here.
  declare_string("input_reference_urdf_path", "");
  declare_string("model_action_follower_srdf_path", "");
  declare_string("follower_joint_states_topic", "/joint_states");
  declare_string(
    "right_leader_topic",
    "/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory");
  declare_string(
    "left_leader_topic",
    "/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory");
  declare_string(
    "right_command_topic",
    "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory");
  declare_string(
    "left_command_topic",
    "/leader/joint_trajectory_command_broadcaster_left/joint_trajectory");
  declare_string("right_gripper_joint", "gripper_r_joint1");
  declare_string("left_gripper_joint", "gripper_l_joint1");
  // Follower grippers are optional; leader triggers remain independent inputs.
  const auto right_gripper = node_->get_parameter(parameterName("right_gripper_joint")).as_string();
  const auto left_gripper = node_->get_parameter(parameterName("left_gripper_joint")).as_string();
  declare_string("leader_right_gripper_joint",
    right_gripper.empty() ? "gripper_r_joint1" : right_gripper);
  declare_string("leader_left_gripper_joint",
    left_gripper.empty() ? "gripper_l_joint1" : left_gripper);
  declare_string("follower_right_eef", "arm_r_link7");
  declare_string("follower_left_eef", "arm_l_link7");
  declare_string("follower_base_frame", "base_link");
  declare_string("follower_right_eef_pose_topic", "~/follower/right/eef_pose");
  declare_string("follower_left_eef_pose_topic", "~/follower/left/eef_pose");
  declare_string(
    "follower_right_eef_reference_topic", "~/follower/right/reference_eef_pose");
  declare_string(
    "follower_left_eef_reference_topic", "~/follower/left/reference_eef_pose");
  const auto publish_follower_eef_state_parameter =
    parameterName("publish_follower_eef_state");
  if (!node_->has_parameter(publish_follower_eef_state_parameter)) {
    node_->declare_parameter(publish_follower_eef_state_parameter, true);
  }
  const auto publish_eef_pose_references_parameter =
    parameterName("publish_eef_pose_references");
  if (!node_->has_parameter(publish_eef_pose_references_parameter)) {
    node_->declare_parameter(publish_eef_pose_references_parameter, true);
  }
  publish_follower_eef_state_ =
    node_->get_parameter(publish_follower_eef_state_parameter).as_bool();
  publish_eef_pose_references_ =
    node_->get_parameter(publish_eef_pose_references_parameter).as_bool();

  follower_joint_states_topic_ =
    node_->get_parameter(parameterName("follower_joint_states_topic")).as_string();
  leader_input_channels_ = {
    JointTrajectoryChannel{
      kLeftGroupId, node_->get_parameter(parameterName("left_leader_topic")).as_string()},
    JointTrajectoryChannel{
      kRightGroupId, node_->get_parameter(parameterName("right_leader_topic")).as_string()}};
  if (!initialize()) {
    return false;
  }
  return control_interface_.configure(
    node, parameter_prefix, mode_configuration_.control_groups,
    std::move(request_callback));
}

bool AIWorkerTeleoperation::initialize()
{
  const auto follower_urdf =
    node_->get_parameter(parameterName("follower_urdf_path")).as_string();
  const auto follower_srdf =
    node_->get_parameter(parameterName("follower_srdf_path")).as_string();
  if (follower_urdf.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Follower URDF path is required");
    return false;
  }

  follower_kinematics_ =
    std::make_shared<cyclo_motion_controller::kinematics::KinematicsSolver>(
    follower_urdf, follower_srdf);
  active_srdf_path_ = follower_srdf;

  follower_joint_names_ = follower_kinematics_->getJointNames();
  for (size_t i = 0; i < follower_joint_names_.size(); ++i) {
    follower_index_[follower_joint_names_[i]] = static_cast<int>(i);
  }

  for (const auto & name : follower_joint_names_) {
    const int index = follower_index_.at(name);
    if (name.find("arm_l_joint") != std::string::npos) {
      left_arm_names_.push_back(name);
      left_arm_indices_.push_back(index);
    } else if (name.find("arm_r_joint") != std::string::npos) {
      right_arm_names_.push_back(name);
      right_arm_indices_.push_back(index);
    }
  }
  if (left_arm_indices_.size() != 7 || right_arm_indices_.size() != 7) {
    RCLCPP_ERROR(
      node_->get_logger(), "AI Worker teleoperation requires 7 left and 7 right arm joints");
    return false;
  }

  const int follower_dof = follower_kinematics_->getDof();
  follower_position_.setZero(follower_dof);
  follower_velocity_.setZero(follower_dof);
  leader_reference_.setZero(follower_dof);

  right_gripper_joint_ =
    node_->get_parameter(parameterName("right_gripper_joint")).as_string();
  left_gripper_joint_ =
    node_->get_parameter(parameterName("left_gripper_joint")).as_string();
  right_leader_gripper_joint_ =
    node_->get_parameter(parameterName("leader_right_gripper_joint")).as_string();
  left_leader_gripper_joint_ =
    node_->get_parameter(parameterName("leader_left_gripper_joint")).as_string();
  follower_base_frame_ =
    node_->get_parameter(parameterName("follower_base_frame")).as_string();

  mode_configuration_.follower_kinematics = follower_kinematics_;
  const auto gripper_joints = [](const std::string & name) {
      std::vector<ControlGroupConfiguration::AuxiliaryJoint> joints;
      if (!name.empty()) {joints.push_back({name, "gripper_position"});}
      return joints;
    };
  mode_configuration_.control_groups = {
    ControlGroupConfiguration{
      kLeftGroupId, "left", left_arm_indices_,
      node_->get_parameter(parameterName("follower_left_eef")).as_string(),
      gripper_joints(left_gripper_joint_), "arm_base_link"},
    ControlGroupConfiguration{
      kRightGroupId, "right", right_arm_indices_,
      node_->get_parameter(parameterName("follower_right_eef")).as_string(),
      gripper_joints(right_gripper_joint_), "arm_base_link"}};
  // Validate the control frame before accepting commands.
  follower_kinematics_->computePose(follower_position_, "arm_base_link");
  control_group_states_.assign(mode_configuration_.control_groups.size(), ControlGroupState{});
  follower_auxiliary_position_.resize(mode_configuration_.control_groups.size());
  leader_auxiliary_reference_.resize(mode_configuration_.control_groups.size());
  for (const auto & group : mode_configuration_.control_groups) {
    follower_auxiliary_position_[group.id] =
      Eigen::VectorXd::Zero(group.auxiliary_joints.size());
    leader_auxiliary_reference_[group.id] =
      Eigen::VectorXd::Zero(group.auxiliary_joints.size());
  }

  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  right_publisher_ =
    node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    node_->get_parameter(parameterName("right_command_topic")).as_string(), command_qos);
  left_publisher_ =
    node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    node_->get_parameter(parameterName("left_command_topic")).as_string(), command_qos);
  if (publish_follower_eef_state_) {
    right_eef_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      node_->get_parameter(parameterName("follower_right_eef_pose_topic")).as_string(),
      rclcpp::SensorDataQoS().keep_last(1));
    left_eef_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      node_->get_parameter(parameterName("follower_left_eef_pose_topic")).as_string(),
      rclcpp::SensorDataQoS().keep_last(1));
  }
  if (publish_eef_pose_references_) {
    // The shared action channel uses the same QoS for teleop output and model input.
    right_eef_reference_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      node_->get_parameter(parameterName("follower_right_eef_reference_topic")).as_string(),
      command_qos);
    left_eef_reference_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      node_->get_parameter(parameterName("follower_left_eef_reference_topic")).as_string(),
      command_qos);
  }
  return true;
}

bool AIWorkerTeleoperation::updateFollowerState(const sensor_msgs::msg::JointState & message)
{
  const auto reject = [this](const std::string & reason) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "Follower feedback rejected: %s", reason.c_str());
      return false;
    };
  if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000u) {
    return reject("invalid timestamp");
  }
  if (
    message.position.size() != message.name.size() ||
    (!message.velocity.empty() && message.velocity.size() != message.name.size()))
  {
    return reject("joint name/position/velocity array sizes differ");
  }

  std::unordered_map<std::string, size_t> message_index;
  for (size_t i = 0; i < message.name.size(); ++i) {
    if (!message_index.emplace(message.name[i], i).second) {
      return reject("duplicate joint '" + message.name[i] + "'");
    }
  }

  Eigen::VectorXd follower_position = follower_position_;
  Eigen::VectorXd follower_velocity = follower_velocity_;
  GroupAuxiliaryPositions follower_auxiliary_position = follower_auxiliary_position_;
  for (size_t i = 0; i < follower_joint_names_.size(); ++i) {
    const auto iter = message_index.find(follower_joint_names_[i]);
    if (iter == message_index.end()) {
      return reject("missing joint '" + follower_joint_names_[i] + "'");
    }
    const size_t source = iter->second;
    if (!std::isfinite(message.position[source])) {
      return reject("non-finite position for '" + follower_joint_names_[i] + "'");
    }
    follower_position[i] = message.position[source];
    if (!message.velocity.empty()) {
      if (!std::isfinite(message.velocity[source])) {
        return reject("non-finite velocity for '" + follower_joint_names_[i] + "'");
      }
      follower_velocity[i] = message.velocity[source];
    } else {
      follower_velocity[i] = 0.0;
    }
  }
  for (const auto & group : mode_configuration_.control_groups) {
    for (size_t i = 0; i < group.auxiliary_joints.size(); ++i) {
      const auto joint = message_index.find(group.auxiliary_joints[i].joint_name);
      if (joint == message_index.end() || !std::isfinite(message.position[joint->second])) {
        return reject("missing or non-finite gripper joint position");
      }
      follower_auxiliary_position[group.id][i] = message.position[joint->second];
    }
  }

  follower_position_ = std::move(follower_position);
  follower_velocity_ = std::move(follower_velocity);
  follower_auxiliary_position_ = std::move(follower_auxiliary_position);
  return true;
}

std::vector<JointTrajectoryChannel> AIWorkerTeleoperation::followerCommandChannels() const
{
  return {
    {kLeftGroupId, node_->get_parameter(parameterName("left_command_topic")).as_string()},
    {kRightGroupId, node_->get_parameter(parameterName("right_command_topic")).as_string()}};
}

void AIWorkerTeleoperation::publishFollowerEefState(const std_msgs::msg::Header & header)
{
  if (publish_follower_eef_state_) {publishFollowerEefPoses(header);}
}

bool AIWorkerTeleoperation::selectControlSource(const std::string & source)
{
  auto srdf = node_->get_parameter(parameterName(
    source ==
      "model_action" ? "model_action_follower_srdf_path" : "follower_srdf_path")).as_string();
  if (srdf.empty()) {
    srdf = node_->get_parameter(parameterName("follower_srdf_path")).as_string();
  }
  if (srdf == active_srdf_path_) {return true;}
  auto kinematics = std::make_shared<cyclo_motion_controller::kinematics::KinematicsSolver>(
    node_->get_parameter(parameterName("follower_urdf_path")).as_string(), srdf);
  if (kinematics->getJointNames() != follower_joint_names_) {return false;}
  kinematics->updateState(follower_position_, follower_velocity_);
  follower_kinematics_ = std::move(kinematics);
  mode_configuration_.follower_kinematics = follower_kinematics_;
  active_srdf_path_ = srdf;
  return true;
}

void AIWorkerTeleoperation::setModeRequestCallback(ModeRequest callback)
{
  control_interface_.setModeRequestCallback(std::move(callback));
}

void AIWorkerTeleoperation::publishFollowerEefPoses(
  const std_msgs::msg::Header & source_header)
{
  follower_kinematics_->updateState(follower_position_, follower_velocity_);
  auto make_message = [this, &source_header](const Eigen::Affine3d & pose) {
      geometry_msgs::msg::PoseStamped message;
      message.header = source_header;
      if (rclcpp::Time(message.header.stamp).nanoseconds() == 0) {
        message.header.stamp = node_->now();
      }
      message.header.frame_id = follower_base_frame_;
      message.pose.position.x = pose.translation().x();
      message.pose.position.y = pose.translation().y();
      message.pose.position.z = pose.translation().z();
      const Eigen::Quaterniond orientation(pose.linear());
      message.pose.orientation.x = orientation.x();
      message.pose.orientation.y = orientation.y();
      message.pose.orientation.z = orientation.z();
      message.pose.orientation.w = orientation.w();
      return message;
    };
  left_eef_pose_publisher_->publish(make_message(
    follower_kinematics_->getPose(mode_configuration_.control_groups.at(
      kLeftGroupId).follower_eef)));
  right_eef_pose_publisher_->publish(make_message(
    follower_kinematics_->getPose(
      mode_configuration_.control_groups.at(kRightGroupId).follower_eef)));
}

bool AIWorkerTeleoperation::updateLeaderReference(
  const trajectory_msgs::msg::JointTrajectory & message,
  const ControlGroupId target_group)
{
  if (target_group != kLeftGroupId && target_group != kRightGroupId) {
    return false;
  }
  if (message.points.empty()) {
    return false;
  }
  const auto & point = message.points.front();
  if (
    point.positions.empty() ||
    point.positions.size() != message.joint_names.size())
  {
    return false;
  }
  const double duration = rclcpp::Duration(point.time_from_start).seconds();
  if (!std::isfinite(duration) || duration < 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Leader trajectory ignored: time_from_start must be >= 0");
    return false;
  }
  const auto & requested_indices =
    target_group == kLeftGroupId ? left_arm_indices_ : right_arm_indices_;
  const auto & requested_names =
    target_group == kLeftGroupId ? left_arm_names_ : right_arm_names_;
  const auto & requested_gripper =
    target_group == kLeftGroupId ? left_leader_gripper_joint_ : right_leader_gripper_joint_;
  const bool requires_gripper = leader_auxiliary_reference_[target_group].size() != 0;
  if (message.joint_names.size() != requested_names.size() + 1 &&
    (requires_gripper || message.joint_names.size() != requested_names.size()))
  {
    return false;
  }

  Eigen::VectorXd leader_reference = leader_reference_;
  GroupAuxiliaryPositions leader_auxiliary_reference = leader_auxiliary_reference_;
  std::unordered_set<std::string> received;
  size_t updated_arm_joints = 0;
  bool updated_gripper = false;
  for (size_t i = 0; i < message.joint_names.size(); ++i) {
    const auto & joint_name = message.joint_names[i];
    const double position = point.positions[i];
    if (!received.insert(joint_name).second || !std::isfinite(position)) {
      return false;
    }

    if (joint_name == requested_gripper) {
      if (requires_gripper) {leader_auxiliary_reference[target_group][0] = position;}
      updated_gripper = true;
      continue;
    }

    if (std::find(requested_names.begin(), requested_names.end(), joint_name) ==
      requested_names.end())
    {
      return false;
    }

    const auto follower = follower_index_.find(joint_name);
    if (follower == follower_index_.end()) {
      return false;
    }
    leader_reference[follower->second] = position;
    ++updated_arm_joints;
  }
  if (updated_arm_joints != requested_indices.size() || (requires_gripper && !updated_gripper)) {
    return false;
  }

  leader_reference_ = std::move(leader_reference);
  leader_auxiliary_reference_ = std::move(leader_auxiliary_reference);
  if (target_group == kLeftGroupId) {
    control_group_states_[kLeftGroupId].leader_duration = duration;
    ++control_group_states_[kLeftGroupId].leader_sequence;
  } else if (target_group == kRightGroupId) {
    control_group_states_[kRightGroupId].leader_duration = duration;
    ++control_group_states_[kRightGroupId].leader_sequence;
  }
  return true;
}

bool AIWorkerTeleoperation::updateGripperReference(
  const trajectory_msgs::msg::JointTrajectory & message,
  const ControlGroupId target_group)
{
  if (
    (target_group != kLeftGroupId && target_group != kRightGroupId) ||
    message.joint_names.size() != 1 || message.points.empty())
  {
    return false;
  }

  const auto & point = message.points.front();
  if (point.positions.size() != 1) {
    return false;
  }
  const double duration = rclcpp::Duration(point.time_from_start).seconds();
  const std::string & expected_gripper =
    target_group == kLeftGroupId ? left_gripper_joint_ : right_gripper_joint_;
  if (
    message.joint_names.front() != expected_gripper ||
    !std::isfinite(point.positions.front()) ||
    !std::isfinite(duration) || duration < 0.0)
  {
    return false;
  }
  if (
    target_group >= leader_auxiliary_reference_.size() ||
    leader_auxiliary_reference_[target_group].size() != 1)
  {
    return false;
  }

  GroupAuxiliaryPositions gripper_reference = leader_auxiliary_reference_;
  gripper_reference[target_group][0] = point.positions.front();
  leader_auxiliary_reference_ = std::move(gripper_reference);
  return true;
}

trajectory_msgs::msg::JointTrajectory AIWorkerTeleoperation::makeArmTrajectory(
  const std::vector<int> & indices,
  const std::vector<std::string> & names,
  const Eigen::VectorXd & command,
  const std::string & gripper_name,
  const Eigen::VectorXd & gripper_position) const
{
  trajectory_msgs::msg::JointTrajectory message;
  message.header.stamp = rclcpp::Time(0, 0);
  message.joint_names = names;
  if (!gripper_name.empty()) {message.joint_names.push_back(gripper_name);}
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.reserve(indices.size() + 1);
  for (const int index : indices) {
    point.positions.push_back(command[index]);
  }
  if (!gripper_name.empty()) {point.positions.push_back(gripper_position[0]);}
  point.time_from_start = rclcpp::Duration(0, 0);
  message.points.push_back(std::move(point));
  return message;
}

void AIWorkerTeleoperation::publish(
  const Eigen::VectorXd & command,
  const GroupAuxiliaryPositions & auxiliary_command)
{
  left_publisher_->publish(makeArmTrajectory(
    left_arm_indices_, left_arm_names_, command,
    left_gripper_joint_, auxiliary_command.at(kLeftGroupId)));
  right_publisher_->publish(makeArmTrajectory(
    right_arm_indices_, right_arm_names_, command,
    right_gripper_joint_, auxiliary_command.at(kRightGroupId)));
}

void AIWorkerTeleoperation::publishEefPoseReferences(
  const Eigen::VectorXd & command,
  const std::vector<EefPoseReference> & references, const ControlGroupMask groups)
{
  if (!publish_eef_pose_references_ || command.size() != follower_position_.size()) {
    return;
  }

  std::unordered_map<ControlGroupId, Eigen::Affine3d> explicit_references;
  for (const auto & reference : references) {
    if (reference.pose.matrix().allFinite()) {
      explicit_references[reference.group_id] = reference.pose;
    }
  }

  follower_kinematics_->updateState(
    command, Eigen::VectorXd::Zero(command.size()));
  for (const auto & group : mode_configuration_.control_groups) {
    if (!containsControlGroup(groups, group.id)) {continue;}
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    if (group.id == kLeftGroupId) {
      publisher = left_eef_reference_publisher_;
    } else if (group.id == kRightGroupId) {
      publisher = right_eef_reference_publisher_;
    } else {
      continue;
    }
    if (!publisher) {
      continue;
    }
    const auto reference = explicit_references.find(group.id);
    const Eigen::Affine3d pose = reference == explicit_references.end() ?
      follower_kinematics_->getPose(group.follower_eef) : reference->second;
    if (!pose.matrix().allFinite()) {
      continue;
    }

    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = node_->now();
    message.header.frame_id = follower_base_frame_;
    message.pose.position.x = pose.translation().x();
    message.pose.position.y = pose.translation().y();
    message.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond orientation(pose.linear());
    message.pose.orientation.x = orientation.x();
    message.pose.orientation.y = orientation.y();
    message.pose.orientation.z = orientation.z();
    message.pose.orientation.w = orientation.w();
    publisher->publish(message);
  }
}

void AIWorkerTeleoperation::publishStatus(const ControlStatus & status)
{
  control_interface_.publishStatus(status);
}
}  // namespace cyclo_teleoperation::robots::ai_worker

PLUGINLIB_EXPORT_CLASS(
  cyclo_teleoperation::robots::ai_worker::AIWorkerTeleoperation,
  cyclo_teleoperation::RobotTeleoperation)
