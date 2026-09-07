// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <commander_behaviors/offset_joint_positions.hpp>

namespace commander_behaviors::test
{
namespace
{
constexpr auto kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <OffsetJointPositions direction="{@direction}"
                          rotation="{@rotation}"
                          current_positions="{@current_positions}"
                          target_positions="{target_positions}"/>
  </BehaviorTree>
</root>)";

/// @brief Tick the node once, with the given command parameters on the blackboard.
BT::Tree makeTree(BT::Blackboard::Ptr blackboard)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<OffsetJointPositions>("OffsetJointPositions");
  factory.registerBehaviorTreeFromText(kTree);
  return factory.createTree("MainTree", BT::Blackboard::create(blackboard));
}
}  // namespace

TEST(OffsetJointPositionsNode, TheJointsAreOffsetByTheCommandedRotation)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("direction", std::string{ "clockwise" });
  blackboard->set("rotation", 6.28);
  blackboard->set("current_positions", std::vector<double>{ 0.0, 1.0 });

  auto tree = makeTree(blackboard);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  const auto targets = tree.rootBlackboard()->get<std::vector<double>>("target_positions");
  ASSERT_EQ(targets.size(), 2u);
  EXPECT_DOUBLE_EQ(targets[0], -6.28);
  EXPECT_DOUBLE_EQ(targets[1], -5.28);
}

TEST(OffsetJointPositionsNode, AnUnknownDirectionIsRejected)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("direction", std::string{ "sideways" });
  blackboard->set("rotation", 1.0);
  blackboard->set("current_positions", std::vector<double>{ 0.0 });

  auto tree = makeTree(blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

TEST(OffsetJointPositionsNode, ANegativeRotationIsRejected)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("direction", std::string{ "clockwise" });
  blackboard->set("rotation", -1.0);
  blackboard->set("current_positions", std::vector<double>{ 0.0 });

  auto tree = makeTree(blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

TEST(OffsetJointPositionsNode, AMissingParameterIsRejected)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("direction", std::string{ "clockwise" });
  blackboard->set("current_positions", std::vector<double>{ 0.0 });

  auto tree = makeTree(blackboard);
  EXPECT_THROW(tree.tickOnce(), BT::RuntimeError);
}

}  // namespace commander_behaviors::test
