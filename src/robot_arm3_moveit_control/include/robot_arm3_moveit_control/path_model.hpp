#pragma once

#include <arm_tcp_bridge_interfaces/action/execute_path.hpp>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace robot_arm3_moveit_control
{

using ExecutePath = arm_tcp_bridge_interfaces::action::ExecutePath;

class PathModelError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

void validate_path_goal(const ExecutePath::Goal& goal, std::size_t max_nodes);

std::vector<std::uint32_t> execution_node_indexes(const ExecutePath::Goal& goal);

std::vector<std::vector<std::uint32_t>> partition_path_at_waits(const ExecutePath::Goal& goal);

}  // namespace robot_arm3_moveit_control
