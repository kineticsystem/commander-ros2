// Copyright 2026 kineticsystem
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <commander_server/payload.hpp>

namespace commander_server::test
{

TEST(Payload, TheRotationCommand)
{
  const auto payload = parsePayload("{joints: [joint1, joint3], direction: clockwise, rotation: 6.28, duration: 2.5}");

  ASSERT_EQ(payload.size(), 4u);
  EXPECT_EQ(std::get<std::vector<std::string>>(payload.at("joints")), (std::vector<std::string>{ "joint1", "joint3" }));
  EXPECT_EQ(std::get<std::string>(payload.at("direction")), "clockwise");
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("rotation")), 6.28);
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("duration")), 2.5);
}

TEST(Payload, JsonIsValidYamlAndIsAccepted)
{
  const auto payload = parsePayload(R"({"direction": "clockwise", "rotation": 1.5})");

  ASSERT_EQ(payload.size(), 2u);
  EXPECT_EQ(std::get<std::string>(payload.at("direction")), "clockwise");
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("rotation")), 1.5);
}

TEST(Payload, AnEmptyPayloadHasNoParameter)
{
  EXPECT_TRUE(parsePayload("").empty());
}

TEST(Payload, NumbersAreParsedAsNumbers)
{
  const auto payload = parsePayload("{a: 1, b: -2.5, c: 1e3}");
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("a")), 1.0);
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("b")), -2.5);
  EXPECT_DOUBLE_EQ(std::get<double>(payload.at("c")), 1000.0);
}

TEST(Payload, AQuotedNumberIsAString)
{
  const auto payload = parsePayload("{a: '1', b: [\"2\", \"3\"]}");
  EXPECT_EQ(std::get<std::string>(payload.at("a")), "1");
  EXPECT_EQ(std::get<std::vector<std::string>>(payload.at("b")), (std::vector<std::string>{ "2", "3" }));
}

TEST(Payload, AListOfNumbersIsAListOfNumbers)
{
  const auto payload = parsePayload("{positions: [0.0, 1.5, -2]}");
  EXPECT_EQ(std::get<std::vector<double>>(payload.at("positions")), (std::vector<double>{ 0.0, 1.5, -2.0 }));
}

TEST(Payload, AnEmptyListIsAListOfStrings)
{
  const auto payload = parsePayload("{joints: []}");
  EXPECT_TRUE(std::get<std::vector<std::string>>(payload.at("joints")).empty());
}

TEST(Payload, TheCommandParametersMustBeAMap)
{
  EXPECT_THROW(parsePayload("[1, 2, 3]"), PayloadError);
  EXPECT_THROW(parsePayload("clockwise"), PayloadError);
}

TEST(Payload, NestedValuesAreNotSupported)
{
  EXPECT_THROW(parsePayload("{joint1: {rotation: 1.0}}"), PayloadError);
}

TEST(Payload, InvalidYamlIsRejected)
{
  EXPECT_THROW(parsePayload("{joints: [joint1"), PayloadError);
}

TEST(Payload, TheParametersAreReadableFromTheBlackboard)
{
  const auto payload = parsePayload("{joints: [joint1], direction: clockwise, rotation: 6.28}");

  auto blackboard = BT::Blackboard::create();
  writeToBlackboard(payload, *blackboard);

  EXPECT_EQ(blackboard->get<std::vector<std::string>>("joints"), (std::vector<std::string>{ "joint1" }));
  EXPECT_EQ(blackboard->get<std::string>("direction"), "clockwise");
  EXPECT_DOUBLE_EQ(blackboard->get<double>("rotation"), 6.28);
}

}  // namespace commander_server::test
