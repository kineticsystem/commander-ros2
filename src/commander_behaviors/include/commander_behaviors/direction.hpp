// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

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
