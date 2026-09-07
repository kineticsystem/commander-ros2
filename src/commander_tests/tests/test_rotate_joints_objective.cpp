// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

// End to end test of the RotateJoints objective: it loads the XML shipped by
// commander_objectives, registers the real behaviors, and runs the tree against
// a fake robot. Only the action server itself (which is provided by
// BehaviorTree.ROS2) is left out.

#include <chrono>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <commander_behaviors/register_nodes.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fake/fake_controller_manager.hpp"
#include "fake/fake_robot.hpp"
#include "objective.hpp"

namespace commander_tests
{
namespace
{
constexpr auto kJointStateTopic = "/joint_states";
constexpr auto kActionName = "/joint_trajectory_controller/follow_joint_trajectory";
constexpr auto kObjective = "RotateJoints";

/// @brief The joints of the StepIt robot, and where they start from.
const std::vector<std::string> kJointNames{ "joint1", "joint2", "joint3", "joint4", "joint5" };
const std::vector<double> kJointPositions{ 0.5, 1.0, 0.0, -1.0, 2.0 };

}  // namespace

class RotateJointsObjective : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("commander_tests");
    robot_ = std::make_unique<FakeRobot>(kJointStateTopic, kActionName, kJointNames, kJointPositions);

    // The objective makes sure the trajectory controller is the one driving the
    // robot before it moves, so the controller manager has to answer too.
    manager_ = std::make_unique<FakeControllerManager>(
        std::vector<FakeControllerManager::Controller>{ { "joint_trajectory_controller", "inactive", true },
                                                        { "joint_state_broadcaster", "active", false },
                                                        { "velocity_controller", "active", true } });

    BT::RosNodeParams params;
    params.nh = node_;
    params.server_timeout = std::chrono::milliseconds{ 2000 };
    params.wait_for_server_timeout = std::chrono::milliseconds{ 2000 };

    commander_behaviors::registerNodes(factory_, params);
    factory_.registerBehaviorTreeFromFile(treePath("subtrees", "ensure_controllers.xml").string());
    factory_.registerBehaviorTreeFromFile(treePath("objectives", "rotate_joints.xml").string());
  }

  void TearDown() override
  {
    manager_.reset();
    robot_.reset();
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FakeRobot> robot_;
  std::unique_ptr<FakeControllerManager> manager_;
  BT::BehaviorTreeFactory factory_;
};

TEST_F(RotateJointsObjective, ClockwiseRotationOfTwoJoints)
{
  ASSERT_EQ(runObjective(factory_, kObjective,
                         "{joints: [joint1, joint2], direction: clockwise, "
                         "rotation: 6.28, duration: 2.0}"),
            BT::NodeStatus::SUCCESS);

  const auto trajectory = robot_->lastTrajectory();
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_EQ(trajectory->joint_names, (std::vector<std::string>{ "joint1", "joint2" }));

  ASSERT_EQ(trajectory->points.size(), 1u);
  const auto& point = trajectory->points.front();
  ASSERT_EQ(point.positions.size(), 2u);
  // A clockwise rotation decreases the joint position.
  EXPECT_DOUBLE_EQ(point.positions[0], 0.5 - 6.28);
  EXPECT_DOUBLE_EQ(point.positions[1], 1.0 - 6.28);
  EXPECT_EQ(point.velocities, (std::vector<double>{ 0.0, 0.0 }));
  EXPECT_EQ(point.time_from_start.sec, 2);
  EXPECT_EQ(point.time_from_start.nanosec, 0u);
}

TEST_F(RotateJointsObjective, CounterClockwiseRotationOfOneJoint)
{
  ASSERT_EQ(runObjective(factory_, kObjective,
                         "{joints: [joint4], direction: counterclockwise, "
                         "rotation: 1.57}"),
            BT::NodeStatus::SUCCESS);

  const auto trajectory = robot_->lastTrajectory();
  ASSERT_TRUE(trajectory.has_value());
  EXPECT_EQ(trajectory->joint_names, (std::vector<std::string>{ "joint4" }));
  ASSERT_EQ(trajectory->points.size(), 1u);
  ASSERT_EQ(trajectory->points.front().positions.size(), 1u);
  EXPECT_DOUBLE_EQ(trajectory->points.front().positions[0], -1.0 + 1.57);
  // The duration is optional and falls back to its default.
  EXPECT_EQ(trajectory->points.front().time_from_start.sec, 5);
}

// The objective needs the trajectory controller: it activates it, and stops
// whatever else was driving the robot.
TEST_F(RotateJointsObjective, TheTrajectoryControllerIsActivatedBeforeMoving)
{
  ASSERT_EQ(runObjective(factory_, kObjective, "{joints: [joint1], direction: clockwise, rotation: 1.0}"),
            BT::NodeStatus::SUCCESS);

  EXPECT_EQ(manager_->stateOf("joint_trajectory_controller"), "active");
  EXPECT_EQ(manager_->stateOf("velocity_controller"), "inactive");
  EXPECT_EQ(manager_->stateOf("joint_state_broadcaster"), "active");
}

TEST_F(RotateJointsObjective, AnUnknownJointFailsTheObjective)
{
  EXPECT_EQ(runObjective(factory_, kObjective, "{joints: [joint9], direction: clockwise, rotation: 1.0}"),
            BT::NodeStatus::FAILURE);
  EXPECT_FALSE(robot_->lastTrajectory().has_value());
}

TEST_F(RotateJointsObjective, AControllerErrorFailsTheObjective)
{
  robot_->failNextTrajectory();

  EXPECT_EQ(runObjective(factory_, kObjective, "{joints: [joint1], direction: clockwise, rotation: 1.0}"),
            BT::NodeStatus::FAILURE);
  EXPECT_TRUE(robot_->lastTrajectory().has_value());
}

}  // namespace commander_tests
