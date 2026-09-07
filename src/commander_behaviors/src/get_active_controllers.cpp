// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include "commander_behaviors/get_active_controllers.hpp"

#include <algorithm>

#include "commander_behaviors/ports.hpp"

namespace commander_behaviors
{

GetActiveControllers::GetActiveControllers(const std::string& name, const BT::NodeConfig& config,
                                           const BT::RosNodeParams& params)
  : BT::RosServiceNode<controller_manager_msgs::srv::ListControllers>(name, config, params)
{
}

BT::PortsList GetActiveControllers::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::vector<std::string>>("exclude", "controllers to leave out of the result"),
      BT::OutputPort<std::vector<std::string>>("controllers", "active controllers that own a command interface"),
  });
}

bool GetActiveControllers::setRequest(Request::SharedPtr& /* request */)
{
  // The service takes no argument.
  return true;
}

BT::NodeStatus GetActiveControllers::onResponseReceived(const Response::SharedPtr& response)
{
  const auto excluded = getNames(*this, "exclude");

  std::vector<std::string> controllers;
  for (const auto& controller : response->controller)
  {
    if (controller.state != "active" || controller.claimed_interfaces.empty())
    {
      continue;
    }
    if (std::find(excluded.cbegin(), excluded.cend(), controller.name) != excluded.cend())
    {
      continue;
    }
    controllers.push_back(controller.name);
  }

  RCLCPP_INFO(logger(), "%s: %zu controller(s) are driving the robot", name().c_str(), controllers.size());

  setOutput("controllers", controllers);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetActiveControllers::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace commander_behaviors
