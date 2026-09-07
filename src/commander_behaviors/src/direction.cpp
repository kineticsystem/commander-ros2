// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include "commander_behaviors/direction.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace commander_behaviors
{
namespace
{
/// @brief Lowercase the text and drop the separators, so that "counter-clockwise",
/// "counter_clockwise" and "CounterClockwise" are all the same word.
std::string normalize(const std::string& text)
{
  std::string out;
  out.reserve(text.size());
  for (const char c : text)
  {
    if (c == '_' || c == '-' || c == ' ')
    {
      continue;
    }
    out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return out;
}
}  // namespace

std::optional<Direction> parseDirection(const std::string& text)
{
  const std::string word = normalize(text);
  if (word == "clockwise" || word == "cw")
  {
    return Direction::Clockwise;
  }
  if (word == "counterclockwise" || word == "ccw" || word == "anticlockwise")
  {
    return Direction::CounterClockwise;
  }
  return std::nullopt;
}

std::string toString(Direction direction)
{
  return direction == Direction::Clockwise ? "clockwise" : "counterclockwise";
}

double signedRotation(Direction direction, double rotation)
{
  if (rotation < 0.0)
  {
    throw std::invalid_argument("the rotation must not be negative: use the direction "
                                "to rotate the other way around");
  }
  return direction == Direction::Clockwise ? -rotation : rotation;
}

std::vector<double> rotateBy(const std::vector<double>& current_positions, Direction direction, double rotation)
{
  const double offset = signedRotation(direction, rotation);
  std::vector<double> targets;
  targets.reserve(current_positions.size());
  std::transform(current_positions.cbegin(), current_positions.cend(), std::back_inserter(targets),
                 [offset](double position) { return position + offset; });
  return targets;
}

}  // namespace commander_behaviors
