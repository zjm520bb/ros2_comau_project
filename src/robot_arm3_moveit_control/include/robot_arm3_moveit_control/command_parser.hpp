#pragma once

#include <stdexcept>
#include <string>
#include <vector>

namespace robot_arm3_moveit_control
{

enum class CommandType
{
  HELLO,
  GET_POSE,
  GET_JOINTS,
  GET_FLY_QUEUE,
  GET_MOTION_SETTINGS,
  SET_BASE,
  SET_USER_FRAME,
  SET_TOOL,
  SET_ORIENTATION,
  SET_SPEED_JOINT,
  SET_JOINT_OVERRIDES,
  SET_SPEED_LINEAR,
  SET_ACCELERATION,
  SET_DECELERATION,
  MOVE_JOINT,
  MOVE_LINEAR,
  MOVE_CIRCULAR,
  MOVE_JOINT_AUTO,
  MOVE_POSE_AUTO,
  MOVE_RELATIVE,
  MOVE_ABOUT,
  SET_FLY_CART,
  SET_FLY_NORM,
  CLEAR_FLY_QUEUE,
  ADD_FLY_LINEAR,
  ADD_FLY_CIRCULAR,
  ADD_FLY_JOINT,
  EXECUTE_FLY_QUEUE,
};

struct ParsedCommand
{
  CommandType type;
  std::vector<double> values;
};

class CommandParseError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

ParsedCommand parse_command(const std::string& text);

}  // namespace robot_arm3_moveit_control
