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

#include "commander_behaviors/offset_joint_positions.hpp"

#include "commander_behaviors/direction.hpp"

namespace commander_behaviors
{

OffsetJointPositions::OffsetJointPositions(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList OffsetJointPositions::providedPorts()
{
  return {
    BT::InputPort<std::string>("direction", "clockwise or counterclockwise"),
    BT::InputPort<double>("rotation", "rotation amplitude in radians (not negative)"),
    BT::InputPort<std::vector<double>>("current_positions", "joint positions the motion starts from"),
    BT::OutputPort<std::vector<double>>("target_positions", "absolute joint positions to reach"),
  };
}

BT::NodeStatus OffsetJointPositions::tick()
{
  const auto direction_text = getInput<std::string>("direction");
  if (!direction_text)
  {
    throw BT::RuntimeError("OffsetJointPositions: ", direction_text.error());
  }
  const auto direction = parseDirection(direction_text.value());
  if (!direction)
  {
    throw BT::RuntimeError("OffsetJointPositions: unknown direction '", direction_text.value(),
                           "': expected clockwise or counterclockwise");
  }

  const auto rotation = getInput<double>("rotation");
  if (!rotation)
  {
    throw BT::RuntimeError("OffsetJointPositions: ", rotation.error());
  }
  if (rotation.value() < 0.0)
  {
    throw BT::RuntimeError("OffsetJointPositions: the rotation must not be negative: "
                           "use the direction to rotate the other way around");
  }

  const auto current_positions = getInput<std::vector<double>>("current_positions");
  if (!current_positions)
  {
    throw BT::RuntimeError("OffsetJointPositions: ", current_positions.error());
  }

  setOutput("target_positions", rotateBy(current_positions.value(), direction.value(), rotation.value()));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace commander_behaviors
