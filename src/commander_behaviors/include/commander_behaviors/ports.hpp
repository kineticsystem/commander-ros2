// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <string>
#include <vector>

#include <behaviortree_cpp/tree_node.h>

namespace commander_behaviors
{

/**
 * @brief Read a port that holds either a list of names or a single name.
 *
 * A command carrying one controller can then be written as
 * `{controllers: velocity_controller}` as well as
 * `{controllers: [velocity_controller]}`.
 *
 * @return The names, empty if the port is not set.
 */
std::vector<std::string> getNames(const BT::TreeNode& node, const std::string& port);

}  // namespace commander_behaviors
