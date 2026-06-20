#include "robot_arm3_moveit_control/command_parser.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <unordered_map>

namespace robot_arm3_moveit_control
{
namespace
{

std::string trim(const std::string& input)
{
  const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char character) {
    return std::isspace(character) != 0;
  });
  const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char character) {
    return std::isspace(character) != 0;
  }).base();
  return first < last ? std::string(first, last) : std::string();
}

std::vector<double> parse_values(const std::string& payload, std::size_t expected_count)
{
  std::vector<double> values;
  std::stringstream stream(payload);
  std::string token;

  while (std::getline(stream, token, ','))
  {
    token = trim(token);
    if (token.empty())
      throw CommandParseError("Command contains an empty parameter");

    std::size_t parsed_length = 0;
    double value = 0.0;
    try
    {
      value = std::stod(token, &parsed_length);
    }
    catch (const std::exception&)
    {
      throw CommandParseError("Invalid numeric parameter: " + token);
    }

    if (parsed_length != token.size() || !std::isfinite(value))
      throw CommandParseError("Invalid numeric parameter: " + token);
    values.push_back(value);
  }

  if (values.size() != expected_count)
  {
    throw CommandParseError("Expected " + std::to_string(expected_count) + " parameters, received " +
                            std::to_string(values.size()));
  }
  return values;
}

}  // namespace

ParsedCommand parse_command(const std::string& text)
{
  const std::string command = trim(text);
  if (command.empty())
    throw CommandParseError("Command is empty");

  if (command == "Hello")
    return { CommandType::HELLO, {} };
  if (command == "getPose")
    return { CommandType::GET_POSE, {} };
  if (command == "getJoints")
    return { CommandType::GET_JOINTS, {} };
  if (command == "clearFlyQueue")
    return { CommandType::CLEAR_FLY_QUEUE, {} };
  if (command == "executeFlyQueue")
    return { CommandType::EXECUTE_FLY_QUEUE, {} };

  const std::size_t separator = command.find(':');
  if (separator == std::string::npos)
    throw CommandParseError("Unsupported command: " + command);

  const std::string name = trim(command.substr(0, separator));
  const std::string payload = command.substr(separator + 1);

  if (name == "setOrientation")
    return { CommandType::SET_ORIENTATION, parse_values(payload, 1) };
  if (name == "setSpeedJnt")
    return { CommandType::SET_SPEED_JOINT, parse_values(payload, 1) };
  if (name == "setJointOverrides")
    return { CommandType::SET_JOINT_OVERRIDES, parse_values(payload, 6) };
  if (name == "setSpeedLin")
    return { CommandType::SET_SPEED_LINEAR, parse_values(payload, 1) };
  if (name == "setAcceleration")
    return { CommandType::SET_ACCELERATION, parse_values(payload, 1) };
  if (name == "setDeceleration")
    return { CommandType::SET_DECELERATION, parse_values(payload, 1) };
  if (name == "moveJoint")
    return { CommandType::MOVE_JOINT, parse_values(payload, 6) };
  if (name == "moveLin")
    return { CommandType::MOVE_LINEAR, parse_values(payload, 6) };
  if (name == "moveCircular")
    return { CommandType::MOVE_CIRCULAR, parse_values(payload, 12) };
  if (name == "setFlyCart")
    return { CommandType::SET_FLY_CART, parse_values(payload, 3) };
  if (name == "setFlyNorm")
    return { CommandType::SET_FLY_NORM, parse_values(payload, 1) };
  if (name == "addFlyLin")
    return { CommandType::ADD_FLY_LINEAR, parse_values(payload, 6) };
  if (name == "addFlyCirc" || name == "addFlyCircular")
    return { CommandType::ADD_FLY_CIRCULAR, parse_values(payload, 12) };
  if (name == "addFlyJoint")
    return { CommandType::ADD_FLY_JOINT, parse_values(payload, 6) };

  throw CommandParseError("Unsupported command: " + name);
}

}  // namespace robot_arm3_moveit_control
