// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_ros2/tree_execution_server.hpp>

#include "commander_server/payload.hpp"

namespace commander_server
{

/**
 * @brief The single action server through which the robot is commanded.
 *
 * A client sends the name of an objective (a behavior tree published by
 * commander_objectives, or by any other package listed in the parameter
 * `behavior_trees`) together with a payload holding its parameters. The payload
 * is copied into the global blackboard of the tree, where the behaviors read it
 * through the '@' prefix, e.g. {@rotation}.
 */
class CommanderServer : public BT::TreeExecutionServer
{
public:
  explicit CommanderServer(const rclcpp::NodeOptions& options);

protected:
  /// @brief Reject the goal when its payload cannot be understood.
  bool onGoalReceived(const std::string& tree_name, const std::string& payload) override;

  /// @brief Publish the parameters of the command into the global blackboard.
  void onTreeCreated(BT::Tree& tree) override;

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override;

private:
  /// @brief Parameters of the goal being executed.
  Payload payload_;
  /// @brief Blackboard entries written for the previous goal.
  std::vector<std::string> written_keys_;
  std::shared_ptr<BT::StdCoutLogger> logger_;
};

}  // namespace commander_server
