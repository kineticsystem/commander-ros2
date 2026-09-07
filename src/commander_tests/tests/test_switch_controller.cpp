// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <commander_behaviors/switch_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

namespace commander_behaviors::test
{

using Request = controller_manager_msgs::srv::SwitchController::Request;

TEST(Strictness, TheStrictnessOfTheSwitchIsNamed)
{
  EXPECT_EQ(parseStrictness("best_effort"), Request::BEST_EFFORT);
  EXPECT_EQ(parseStrictness("strict"), Request::STRICT);
  EXPECT_EQ(parseStrictness("auto"), Request::AUTO);
  EXPECT_EQ(parseStrictness("force_auto"), Request::FORCE_AUTO);
  EXPECT_EQ(parseStrictness("BEST_EFFORT"), Request::BEST_EFFORT);
}

TEST(Strictness, AnUnknownStrictnessIsRejected)
{
  EXPECT_FALSE(parseStrictness("whenever").has_value());
  EXPECT_FALSE(parseStrictness("").has_value());
}

}  // namespace commander_behaviors::test
