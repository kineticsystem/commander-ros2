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

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace commander_behaviors
{

/**
 * @brief The direction a joint is asked to rotate into.
 *
 * The sign convention of the robot is defined in one single place, here:
 * a clockwise rotation decreases the joint position, as documented in the
 * StepIt README ("rotate joint1 by 6.28 rad clockwise" -> position -6.28).
 */
enum class Direction
{
  Clockwise,
  CounterClockwise
};

/**
 * @brief Parse a rotation direction.
 *
 * Accepted spellings (case insensitive): "clockwise", "cw", "counterclockwise",
 * "counter_clockwise", "counter-clockwise", "ccw".
 *
 * @return std::nullopt if the text is not a known direction.
 */
std::optional<Direction> parseDirection(const std::string& text);

/// @brief Human readable name of a direction, i.e. the canonical spelling.
std::string toString(Direction direction);

/**
 * @brief Apply the sign convention of the robot to a rotation.
 *
 * @param direction  The direction of the rotation.
 * @param rotation   The rotation amplitude in radians. It must not be negative.
 * @return The signed joint displacement, in radians.
 */
double signedRotation(Direction direction, double rotation);

/**
 * @brief Offset each of the given joint positions by the same signed rotation.
 *
 * @param current_positions The joint positions the motion starts from, in radians.
 * @param direction         The direction of the rotation.
 * @param rotation          The rotation amplitude in radians. It must not be negative.
 * @return The absolute target positions, in radians.
 */
std::vector<double> rotateBy(const std::vector<double>& current_positions, Direction direction, double rotation);

}  // namespace commander_behaviors
