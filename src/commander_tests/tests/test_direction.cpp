// Copyright 2026 Giovanni Remigi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

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
