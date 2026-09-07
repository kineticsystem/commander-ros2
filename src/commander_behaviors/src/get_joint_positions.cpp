// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include "commander_behaviors/get_joint_positions.hpp"

#include <algorithm>

namespace commander_behaviors
{

GetJointPositions::GetJointPositions(const std::string& name, const BT::NodeConfig& config,
                                     const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<sensor_msgs::msg::JointState>(name, config, params)
{
}

BT::PortsList GetJointPositions::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::vector<std::string>>("joint_names", "joints to read"),
      BT::OutputPort<std::vector<double>>("positions", "current joint positions, in radians"),
  });
}

BT::NodeStatus GetJointPositions::onTick(const std::shared_ptr<sensor_msgs::msg::JointState>& msg)
{
  if (!msg)
  {
    RCLCPP_DEBUG(logger(), "%s: no joint state received yet", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto joint_names = getInput<std::vector<std::string>>("joint_names");
  if (!joint_names)
  {
    throw BT::RuntimeError("GetJointPositions: ", joint_names.error());
  }

  std::vector<double> positions;
  positions.reserve(joint_names.value().size());
  for (const auto& joint_name : joint_names.value())
  {
    const auto it = std::find(msg->name.cbegin(), msg->name.cend(), joint_name);
    if (it == msg->name.cend())
    {
      RCLCPP_ERROR(logger(), "%s: joint '%s' is not published on the joint state topic", name().c_str(),
                   joint_name.c_str());
      return BT::NodeStatus::FAILURE;
    }
    const auto index = static_cast<std::size_t>(std::distance(msg->name.cbegin(), it));
    if (index >= msg->position.size())
    {
      RCLCPP_ERROR(logger(), "%s: no position published for joint '%s'", name().c_str(), joint_name.c_str());
      return BT::NodeStatus::FAILURE;
    }
    positions.push_back(msg->position[index]);
  }

  setOutput("positions", positions);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace commander_behaviors
