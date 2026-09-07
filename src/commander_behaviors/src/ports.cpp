// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include "commander_behaviors/ports.hpp"

namespace commander_behaviors
{

std::vector<std::string> getNames(const BT::TreeNode& node, const std::string& port)
{
  // getInput never throws: it reports a missing entry, or a type that does not
  // match, as an error, which is exactly what we fall back on here.
  if (const auto names = node.getInput<std::vector<std::string>>(port))
  {
    return names.value();
  }
  if (const auto name = node.getInput<std::string>(port); name && !name.value().empty())
  {
    return { name.value() };
  }
  return {};
}

}  // namespace commander_behaviors
