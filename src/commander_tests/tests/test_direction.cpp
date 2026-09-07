// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <commander_behaviors/direction.hpp>

namespace commander_behaviors::test
{

TEST(Direction, ParseClockwise)
{
  for (const auto& text : { "clockwise", "Clockwise", "CLOCKWISE", "cw", "CW" })
  {
    const auto direction = parseDirection(text);
    ASSERT_TRUE(direction.has_value()) << text;
    EXPECT_EQ(*direction, Direction::Clockwise) << text;
  }
}

TEST(Direction, ParseCounterClockwise)
{
  for (const auto& text :
       { "counterclockwise", "counter_clockwise", "counter-clockwise", "CounterClockwise", "ccw", "anticlockwise" })
  {
    const auto direction = parseDirection(text);
    ASSERT_TRUE(direction.has_value()) << text;
    EXPECT_EQ(*direction, Direction::CounterClockwise) << text;
  }
}

TEST(Direction, ParseUnknownDirection)
{
  EXPECT_FALSE(parseDirection("sideways").has_value());
  EXPECT_FALSE(parseDirection("").has_value());
}

// The robot turns clockwise when the joint position decreases.
TEST(Direction, ClockwiseIsNegative)
{
  EXPECT_DOUBLE_EQ(signedRotation(Direction::Clockwise, 6.28), -6.28);
  EXPECT_DOUBLE_EQ(signedRotation(Direction::CounterClockwise, 6.28), 6.28);
  EXPECT_DOUBLE_EQ(signedRotation(Direction::Clockwise, 0.0), 0.0);
}

TEST(Direction, ANegativeRotationIsRejected)
{
  EXPECT_THROW(signedRotation(Direction::Clockwise, -1.0), std::invalid_argument);
}

TEST(Direction, EveryJointIsOffsetByTheSameRotation)
{
  const std::vector<double> current{ 0.0, 1.0, -2.0 };

  const auto clockwise = rotateBy(current, Direction::Clockwise, 0.5);
  ASSERT_EQ(clockwise.size(), 3u);
  EXPECT_DOUBLE_EQ(clockwise[0], -0.5);
  EXPECT_DOUBLE_EQ(clockwise[1], 0.5);
  EXPECT_DOUBLE_EQ(clockwise[2], -2.5);

  const auto counter_clockwise = rotateBy(current, Direction::CounterClockwise, 0.5);
  ASSERT_EQ(counter_clockwise.size(), 3u);
  EXPECT_DOUBLE_EQ(counter_clockwise[0], 0.5);
  EXPECT_DOUBLE_EQ(counter_clockwise[1], 1.5);
  EXPECT_DOUBLE_EQ(counter_clockwise[2], -1.5);
}

TEST(Direction, NoJointToRotate)
{
  EXPECT_TRUE(rotateBy({}, Direction::Clockwise, 1.0).empty());
}

}  // namespace commander_behaviors::test
