// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include "commander_behaviors/switch_controller.hpp"

#include <algorithm>
#include <cctype>

#include "commander_behaviors/ports.hpp"

namespace commander_behaviors
{
namespace
{
using Srv = controller_manager_msgs::srv::SwitchController;

std::string lowercase(const std::string& text)
{
  std::string out;
  out.reserve(text.size());
  std::transform(text.cbegin(), text.cend(), std::back_inserter(out),
                 [](char c) { return static_cast<char>(std::tolower(static_cast<unsigned char>(c))); });
  return out;
}

std::string join(const std::vector<std::string>& names)
{
  std::string out;
  for (const auto& name : names)
  {
    if (!out.empty())
    {
      out += ", ";
    }
    out += name;
  }
  return out.empty() ? "none" : out;
}
}  // namespace

std::optional<std::int32_t> parseStrictness(const std::string& text)
{
  const auto word = lowercase(text);
  if (word == "best_effort")
  {
    return Srv::Request::BEST_EFFORT;
  }
  if (word == "strict")
  {
    return Srv::Request::STRICT;
  }
  if (word == "auto")
  {
    return Srv::Request::AUTO;
  }
  if (word == "force_auto")
  {
    return Srv::Request::FORCE_AUTO;
  }
  return std::nullopt;
}

SwitchController::SwitchController(const std::string& name, const BT::NodeConfig& config,
                                   const BT::RosNodeParams& params)
  : BT::RosServiceNode<controller_manager_msgs::srv::SwitchController>(name, config, params)
{
}

BT::PortsList SwitchController::providedPorts()
{
  return providedBasicPorts({
      BT::InputPort<std::vector<std::string>>("activate", "controllers to activate"),
      BT::InputPort<std::vector<std::string>>("deactivate", "controllers to deactivate"),
      BT::InputPort<std::string>("strictness", "best_effort", "best_effort, strict, auto or force_auto"),
  });
}

bool SwitchController::setRequest(Request::SharedPtr& request)
{
  const auto activate = getNames(*this, "activate");
  const auto deactivate = getNames(*this, "deactivate");

  if (activate.empty() && deactivate.empty())
  {
    throw BT::RuntimeError("SwitchController: no controller to activate or deactivate");
  }

  const auto strictness_text = getInput<std::string>("strictness").value_or("best_effort");
  const auto strictness = parseStrictness(strictness_text);
  if (!strictness)
  {
    throw BT::RuntimeError("SwitchController: unknown strictness '", strictness_text,
                           "': expected best_effort, strict, auto or force_auto");
  }

  request->activate_controllers = activate;
  request->deactivate_controllers = deactivate;
  request->strictness = strictness.value();
  request->activate_asap = false;
  // Zero means the controller manager waits as long as it takes.
  request->timeout = rclcpp::Duration::from_seconds(0.0);

  RCLCPP_INFO(logger(), "%s: activating [%s], deactivating [%s]", name().c_str(), join(activate).c_str(),
              join(deactivate).c_str());

  return true;
}

BT::NodeStatus SwitchController::onResponseReceived(const Response::SharedPtr& response)
{
  if (!response->ok)
  {
    RCLCPP_ERROR(logger(), "%s: the controllers were not switched: %s", name().c_str(), response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(logger(), "%s: %s", name().c_str(), response->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SwitchController::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

}  // namespace commander_behaviors
