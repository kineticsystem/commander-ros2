// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

// Entry point used by the commander server to load these behaviors at runtime.
// It must live in a cpp file compiled with BT_PLUGIN_EXPORT (see CMakeLists.txt).

#include "commander_behaviors/register_nodes.hpp"

#include <behaviortree_ros2/plugins.hpp>

BT_REGISTER_ROS_NODES(factory, params)
{
  commander_behaviors::registerNodes(factory, params);
}
