// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace commander_behaviors
{

/**
 * @brief Reads the current position of a set of joints from /joint_states.
 *
 * It returns FAILURE when no message has been received yet, or when one of the
 * requested joints is not part of the message, so that it can be retried by the
 * behavior tree.
 */
class GetJointPositions : public BT::RosTopicSubNode<sensor_msgs::msg::JointState>
{
public:
  GetJointPositions(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::JointState>& msg) override;
};

}  // namespace commander_behaviors
