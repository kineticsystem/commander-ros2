// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include <memory>

#include "commander_server/commander_server.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto server = std::make_shared<commander_server::CommanderServer>(options);

  // The timeout works around a deadlock of the MultiThreadedExecutor, which can
  // happen when publishers or subscribers are added or removed while spinning:
  // the behaviors of a tree do exactly that.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  executor.add_node(server->node());
  executor.spin();
  executor.remove_node(server->node());

  rclcpp::shutdown();

  return 0;
}
