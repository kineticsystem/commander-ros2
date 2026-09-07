// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include "commander_behaviors/register_nodes.hpp"

#include "commander_behaviors/follow_joint_trajectory.hpp"
#include "commander_behaviors/get_active_controllers.hpp"
#include "commander_behaviors/get_joint_positions.hpp"
#include "commander_behaviors/offset_joint_positions.hpp"
#include "commander_behaviors/switch_controller.hpp"

namespace commander_behaviors
{

void registerNodes(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params)
{
  factory.registerNodeType<OffsetJointPositions>("OffsetJointPositions");
  factory.registerNodeType<GetJointPositions>("GetJointPositions", params);
  factory.registerNodeType<FollowJointTrajectory>("FollowJointTrajectory", params);
  factory.registerNodeType<GetActiveControllers>("GetActiveControllers", params);
  factory.registerNodeType<SwitchController>("SwitchController", params);
}

}  // namespace commander_behaviors
