// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <string>
#include <vector>

#include <behaviortree_cpp/action_node.h>

namespace commander_behaviors
{

/**
 * @brief Offsets the current joint positions by the commanded displacement, and
 * writes out the absolute positions to reach.
 *
 * This node moves nothing: it only turns a relative command into the absolute
 * targets that FollowJointTrajectory then sends to the controller.
 *
 * The displacement is currently expressed the way the StepIt motors are
 * commanded, as a rotation in radians plus a direction, but the node is not
 * bound to rotary joints: a prismatic joint only needs a different pair of
 * ports here, and nothing else in the tree changes.
 *
 * This node contains no ROS code at all: it is the piece of logic that knows
 * the sign convention of the robot, and nothing else.
 */
class OffsetJointPositions : public BT::SyncActionNode
{
public:
  OffsetJointPositions(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace commander_behaviors
