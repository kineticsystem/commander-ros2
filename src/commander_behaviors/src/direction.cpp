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
