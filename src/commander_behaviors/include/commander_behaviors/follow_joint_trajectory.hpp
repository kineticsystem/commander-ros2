// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <string>
#include <vector>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace commander_behaviors
{

/**
 * @brief Sends a single-waypoint trajectory to a joint trajectory controller.
 *
 * This is a thin behavior tree wrapper around the FollowJointTrajectory action
 * exposed by the joint_trajectory_controller of the StepIt robot.
 */
class FollowJointTrajectory : public BT::RosActionNode<control_msgs::action::FollowJointTrajectory>
{
public:
  /// @brief Default duration of the motion, in seconds, when no duration is given.
  static constexpr double kDefaultDuration = 5.0;

  FollowJointTrajectory(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& result) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  void onHalt() override;
};

}  // namespace commander_behaviors
