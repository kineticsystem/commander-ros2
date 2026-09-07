// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <string>

#include <behaviortree_ros2/bt_service_node.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace commander_behaviors
{

/**
 * @brief How the controller manager reacts when a switch cannot be honoured.
 *
 * `best_effort` is what a command normally wants: activating a controller that
 * is already active is not an error, while an unknown controller still is.
 */
std::optional<std::int32_t> parseStrictness(const std::string& text);

/**
 * @brief Activates and deactivates controllers, in one single control cycle.
 */
class SwitchController : public BT::RosServiceNode<controller_manager_msgs::srv::SwitchController>
{
public:
  SwitchController(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

}  // namespace commander_behaviors
