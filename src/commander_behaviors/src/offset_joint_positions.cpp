// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

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
