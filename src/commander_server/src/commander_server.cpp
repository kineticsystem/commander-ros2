// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include "commander_server/commander_server.hpp"

namespace commander_server
{

CommanderServer::CommanderServer(const rclcpp::NodeOptions& options) : BT::TreeExecutionServer(options)
{
}

bool CommanderServer::onGoalReceived(const std::string& tree_name, const std::string& payload)
{
  try
  {
    payload_ = parsePayload(payload);
  }
  catch (const PayloadError& ex)
  {
    RCLCPP_ERROR(node()->get_logger(), "Rejecting objective '%s': %s", tree_name.c_str(), ex.what());
    return false;
  }

  RCLCPP_INFO(node()->get_logger(), "Executing objective '%s' with %zu parameter(s)", tree_name.c_str(),
              payload_.size());
  return true;
}

void CommanderServer::onTreeCreated(BT::Tree& tree)
{
  // The parameters of a previous goal must not leak into this one.
  for (const auto& key : written_keys_)
  {
    globalBlackboard()->unset(key);
  }
  written_keys_.clear();

  writeToBlackboard(payload_, *globalBlackboard());
  for (const auto& [key, value] : payload_)
  {
    written_keys_.push_back(key);
  }

  logger_ = std::make_shared<BT::StdCoutLogger>(tree);
}

std::optional<std::string> CommanderServer::onTreeExecutionCompleted(BT::NodeStatus, bool)
{
  logger_.reset();
  return std::nullopt;
}

}  // namespace commander_server
