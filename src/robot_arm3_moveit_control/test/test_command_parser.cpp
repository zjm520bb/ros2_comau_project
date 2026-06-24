#include <gtest/gtest.h>

#include <cmath>

#include "robot_arm3_moveit_control/command_parser.hpp"

namespace control = robot_arm3_moveit_control;

TEST(CommandParser, ParsesSimpleCommands)
{
  EXPECT_EQ(control::parse_command(" Hello ").type, control::CommandType::HELLO);
  EXPECT_EQ(control::parse_command("getPose").type, control::CommandType::GET_POSE);
  EXPECT_EQ(control::parse_command("getJoints").type, control::CommandType::GET_JOINTS);
  EXPECT_EQ(control::parse_command("getFlyQueue").type, control::CommandType::GET_FLY_QUEUE);
  EXPECT_EQ(control::parse_command("getMotionSettings").type, control::CommandType::GET_MOTION_SETTINGS);
  EXPECT_EQ(control::parse_command("clearFlyQueue").type, control::CommandType::CLEAR_FLY_QUEUE);
  EXPECT_EQ(control::parse_command("executeFlyQueue").type, control::CommandType::EXECUTE_FLY_QUEUE);
}

TEST(CommandParser, ParsesMoveJoint)
{
  const auto command = control::parse_command("moveJoint:0,-20,30,0,45,0");
  EXPECT_EQ(command.type, control::CommandType::MOVE_JOINT);
  ASSERT_EQ(command.values.size(), 6u);
  EXPECT_DOUBLE_EQ(command.values[1], -20.0);
  EXPECT_DOUBLE_EQ(command.values[4], 45.0);
}

TEST(CommandParser, ParsesMoveLinear)
{
  const auto command = control::parse_command("moveLin:1000,0,1210,0,90,0");
  EXPECT_EQ(command.type, control::CommandType::MOVE_LINEAR);
  ASSERT_EQ(command.values.size(), 6u);
  EXPECT_DOUBLE_EQ(command.values[0], 1000.0);
  EXPECT_DOUBLE_EQ(command.values[4], 90.0);
}

TEST(CommandParser, ParsesSpeedCommands)
{
  EXPECT_DOUBLE_EQ(control::parse_command("setSpeedJnt:5").values.at(0), 5.0);
  EXPECT_DOUBLE_EQ(control::parse_command("setSpeedLin:0.05").values.at(0), 0.05);
  EXPECT_DOUBLE_EQ(control::parse_command("setAcceleration:10").values.at(0), 10.0);
  EXPECT_DOUBLE_EQ(control::parse_command("setDeceleration:20").values.at(0), 20.0);

  const auto overrides = control::parse_command("setJointOverrides:5,6,7,8,9,10");
  EXPECT_EQ(overrides.type, control::CommandType::SET_JOINT_OVERRIDES);
  ASSERT_EQ(overrides.values.size(), 6u);
  EXPECT_DOUBLE_EQ(overrides.values[2], 7.0);
  EXPECT_DOUBLE_EQ(overrides.values[5], 10.0);
}

TEST(CommandParser, ParsesFrameCommands)
{
  EXPECT_EQ(control::parse_command("setBase:1,2,3,4,5,6").type, control::CommandType::SET_BASE);
  EXPECT_EQ(control::parse_command("setUframe:1,2,3,4,5,6").type, control::CommandType::SET_USER_FRAME);
  EXPECT_EQ(control::parse_command("userFrame:1,2,3,4,5,6").type, control::CommandType::SET_USER_FRAME);
  EXPECT_EQ(control::parse_command("setTool:1,2,3,4,5,6").type, control::CommandType::SET_TOOL);
  EXPECT_EQ(control::parse_command("tool:1,2,3,4,5,6").type, control::CommandType::SET_TOOL);
}

TEST(CommandParser, ParsesMoveCircular)
{
  const auto command = control::parse_command(
      "moveCircular:1,2,3,4,5,6,7,8,9,10,11,12");
  EXPECT_EQ(command.type, control::CommandType::MOVE_CIRCULAR);
  ASSERT_EQ(command.values.size(), 12u);
  EXPECT_DOUBLE_EQ(command.values.front(), 1.0);
  EXPECT_DOUBLE_EQ(command.values.back(), 12.0);
}

TEST(CommandParser, ParsesFlyCommands)
{
  EXPECT_EQ(control::parse_command("setFlyCart:10,0,5").type, control::CommandType::SET_FLY_CART);
  EXPECT_EQ(control::parse_command("setFlyNorm:75").type, control::CommandType::SET_FLY_NORM);
  EXPECT_EQ(control::parse_command("addFlyLin:1,2,3,4,5,6").type, control::CommandType::ADD_FLY_LINEAR);
  EXPECT_EQ(control::parse_command("addFlyJoint:1,2,3,4,5,6").type, control::CommandType::ADD_FLY_JOINT);
  const auto circular = control::parse_command("addFlyCircular:1,2,3,4,5,6,7,8,9,10,11,12");
  EXPECT_EQ(circular.type, control::CommandType::ADD_FLY_CIRCULAR);
  EXPECT_EQ(circular.values.size(), 12u);
}

TEST(CommandParser, RejectsWrongParameterCount)
{
  EXPECT_THROW(control::parse_command("moveJoint:1,2,3"), control::CommandParseError);
  EXPECT_THROW(control::parse_command("moveLin:1,2,3,4,5,6,7"), control::CommandParseError);
  EXPECT_THROW(control::parse_command("setJointOverrides:1,2,3"), control::CommandParseError);
  EXPECT_THROW(control::parse_command("setBase:1,2,3"), control::CommandParseError);
  EXPECT_THROW(control::parse_command("moveCircular:1,2,3,4,5,6"), control::CommandParseError);
  EXPECT_THROW(control::parse_command("setFlyCart:10,0"), control::CommandParseError);
  EXPECT_THROW(control::parse_command("addFlyJoint:1,2,3"), control::CommandParseError);
}

TEST(CommandParser, RejectsInvalidNumbersAndCommands)
{
  EXPECT_THROW(control::parse_command("moveJoint:1,2,nan,4,5,6"), control::CommandParseError);
  EXPECT_THROW(control::parse_command("moveLin:1,2,x,4,5,6"), control::CommandParseError);
  EXPECT_THROW(control::parse_command("moveCircular:1,2,3"), control::CommandParseError);
}
