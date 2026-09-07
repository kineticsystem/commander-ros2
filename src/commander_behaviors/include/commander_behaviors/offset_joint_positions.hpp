// Copyright 2026 Giovanni Remigi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
