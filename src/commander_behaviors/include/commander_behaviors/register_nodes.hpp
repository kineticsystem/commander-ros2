// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/ros_node_params.hpp>

namespace commander_behaviors
{

/**
 * @brief Register every behavior of this package into a factory.
 *
 * The commander server loads these behaviors as a BehaviorTree.CPP plugin, but
 * tests (and any other client that already owns a factory) can call this
 * function directly instead.
 */
void registerNodes(BT::BehaviorTreeFactory& factory, const BT::RosNodeParams& params);

}  // namespace commander_behaviors
