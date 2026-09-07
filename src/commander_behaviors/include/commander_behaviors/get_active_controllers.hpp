// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#pragma once

#include <string>
#include <vector>

#include <behaviortree_ros2/bt_service_node.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

namespace commander_behaviors
{

/**
 * @brief Lists the controllers that are currently driving the robot.
 *
 * Only the active controllers that own at least one command interface are
 * returned: broadcasters, such as the joint_state_broadcaster, only read the
 * state of the robot and must keep running when the controllers are switched.
 */
class GetActiveControllers : public BT::RosServiceNode<controller_manager_msgs::srv::ListControllers>
{
public:
  GetActiveControllers(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

}  // namespace commander_behaviors
