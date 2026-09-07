// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <behaviortree_cpp/blackboard.h>

namespace commander_server
{

/// @brief A value that a command can carry.
using PayloadValue = std::variant<double, std::string, std::vector<double>, std::vector<std::string>>;

/// @brief The parameters of a command, by name.
using Payload = std::map<std::string, PayloadValue>;

/// @brief Thrown when a payload cannot be understood.
class PayloadError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/**
 * @brief Parse the payload of a command.
 *
 * The payload is a YAML (and therefore also JSON) map, for example
 *
 *   {joints: [joint1, joint2], direction: clockwise, rotation: 6.28}
 *
 * Values are typed as follows, so that the behavior tree ports can read them
 * without any further conversion:
 *
 *   - a number             -> double
 *   - any other scalar     -> std::string
 *   - a list of numbers    -> std::vector<double>
 *   - any other list       -> std::vector<std::string>
 *
 * A quoted scalar, such as "5", is always a string.
 *
 * @param text The payload, possibly empty.
 * @return The parameters of the command, empty if the payload is empty.
 * @throw PayloadError if the payload is not a YAML map of scalars and lists.
 */
Payload parsePayload(const std::string& text);

/**
 * @brief Copy the parameters of a command into a blackboard.
 *
 * When the blackboard is the global one, the entries are visible to every tree
 * and subtree through the '@' prefix, e.g. {@rotation}.
 */
void writeToBlackboard(const Payload& payload, BT::Blackboard& blackboard);

}  // namespace commander_server
