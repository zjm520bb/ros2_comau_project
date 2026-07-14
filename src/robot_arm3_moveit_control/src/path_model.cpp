#include "robot_arm3_moveit_control/path_model.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

namespace robot_arm3_moveit_control
{
namespace
{

using PathNode = arm_tcp_bridge_interfaces::msg::PathNode;

bool finite_values(const std::array<double, 6>& values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

}  // namespace

void validate_path_goal(const ExecutePath::Goal& goal, std::size_t max_nodes)
{
  if (goal.path_id == 0 || goal.path_id > 2147483647U)
    throw PathModelError("path_id must fit a positive C4G INTEGER");
  if (goal.path_type != ExecutePath::Goal::CARTESIAN && goal.path_type != ExecutePath::Goal::JOINT)
    throw PathModelError("path_type must be CARTESIAN or JOINT");
  if (goal.nodes.empty())
    throw PathModelError("PATH must contain at least one node");
  if (goal.nodes.size() > max_nodes)
    throw PathModelError("PATH exceeds max_path_nodes");

  const std::uint32_t start = goal.start_index == 0 ? 1 : goal.start_index;
  const std::uint32_t end =
      goal.end_index == 0 ? static_cast<std::uint32_t>(goal.nodes.size()) : goal.end_index;
  if (start < 1 || start > goal.nodes.size() || end < 1 || end > goal.nodes.size())
    throw PathModelError("PATH execution range is invalid");

  std::set<std::uint8_t> frames;
  for (const auto& frame : goal.frames)
  {
    if (frame.index < 1 || frame.index > 7)
      throw PathModelError("PATH frame index must be within 1..7");
    if (!frames.insert(frame.index).second)
      throw PathModelError("PATH contains a duplicate frame index");
    if (!finite_values(frame.pose))
      throw PathModelError("PATH frame contains NaN or infinity");
  }

  std::set<std::uint8_t> condition_slots;
  for (const auto& condition : goal.conditions)
  {
    if (condition.slot < 1 || condition.slot > 32 || condition.handler_id <= 0)
      throw PathModelError("PATH condition slot or handler is invalid");
    if (!condition_slots.insert(condition.slot).second)
      throw PathModelError("PATH contains a duplicate condition slot");
  }

  std::uint8_t previous_type = 255;
  for (std::size_t index = 0; index < goal.nodes.size(); ++index)
  {
    const auto& node = goal.nodes[index];
    if (node.motion_type > PathNode::SEG_VIA)
      throw PathModelError("PATH node has an invalid motion_type");
    if (!finite_values(node.target))
      throw PathModelError("PATH node target contains NaN or infinity");
    if (goal.path_type == ExecutePath::Goal::JOINT && node.motion_type != PathNode::JOINT)
      throw PathModelError("JOINT PATH may contain only JOINT nodes");
    if (goal.path_type == ExecutePath::Goal::CARTESIAN)
    {
      if (node.motion_type == PathNode::CIRCULAR && previous_type != PathNode::SEG_VIA)
        throw PathModelError("CIRCULAR node must follow SEG_VIA");
      if (previous_type == PathNode::SEG_VIA && node.motion_type != PathNode::CIRCULAR)
        throw PathModelError("SEG_VIA node must be followed by CIRCULAR");
      if (node.wait && node.motion_type == PathNode::SEG_VIA)
        throw PathModelError("SEG_VIA node cannot suspend PATH execution");
    }
    if (!std::isfinite(node.linear_speed) || !std::isfinite(node.rotational_speed) ||
        !std::isfinite(node.segment_override) || node.segment_override <= 0.0 ||
        node.segment_override > 100.0 || node.termination_type > 4 ||
        !std::isfinite(node.tolerance) || node.fly_type > 1 ||
        !std::isfinite(node.fly_percent) || !std::isfinite(node.fly_distance_mm) ||
        node.fly_distance_mm < 0.0 || node.fly_trajectory > 3 ||
        !std::isfinite(node.stress_percent) || node.reference_index > 7 || node.tool_index > 7)
      throw PathModelError("PATH node motion parameters are invalid");
    if (node.fly && (node.fly_percent < 1.0 || node.fly_percent > 100.0))
      throw PathModelError("enabled PATH fly_percent must be within 1..100");
    if (goal.path_type == ExecutePath::Goal::JOINT &&
        (node.reference_index != 0 || node.tool_index != 0))
      throw PathModelError("JOINT PATH cannot use Cartesian frames");
    if (node.reference_index != 0 && frames.count(node.reference_index) == 0)
      throw PathModelError("PATH node references an undefined reference frame");
    if (node.tool_index != 0 && frames.count(node.tool_index) == 0)
      throw PathModelError("PATH node references an undefined tool frame");
    const std::uint32_t masks[] = {
      static_cast<std::uint32_t>(node.condition_mask),
      static_cast<std::uint32_t>(node.condition_mask_back)
    };
    for (const auto mask : masks)
      for (std::uint8_t bit = 0; bit < 32; ++bit)
        if ((mask & (std::uint32_t{ 1 } << bit)) != 0 &&
            condition_slots.count(static_cast<std::uint8_t>(bit + 1)) == 0)
          throw PathModelError("PATH node mask selects an undefined condition slot");
    if (goal.path_type == ExecutePath::Goal::JOINT && node.fly && node.fly_type != 0)
      throw PathModelError("JOINT PATH supports only FLY_NORM");
    previous_type = node.motion_type;
  }
  if (previous_type == arm_tcp_bridge_interfaces::msg::PathNode::SEG_VIA)
    throw PathModelError("PATH cannot end with SEG_VIA");
  if (goal.nodes[start - 1].motion_type == arm_tcp_bridge_interfaces::msg::PathNode::CIRCULAR)
    throw PathModelError("PATH range cannot start at CIRCULAR");
  if (goal.nodes[end - 1].motion_type == arm_tcp_bridge_interfaces::msg::PathNode::SEG_VIA)
    throw PathModelError("PATH range cannot end at SEG_VIA");
}

std::vector<std::uint32_t> execution_node_indexes(const ExecutePath::Goal& goal)
{
  const std::uint32_t start = goal.start_index == 0 ? 1 : goal.start_index;
  const std::uint32_t end =
      goal.end_index == 0 ? static_cast<std::uint32_t>(goal.nodes.size()) : goal.end_index;
  std::vector<std::uint32_t> indexes;
  if (start <= end)
  {
    for (std::uint32_t index = start; index <= end; ++index)
      indexes.push_back(index);
  }
  else
  {
    for (std::uint32_t index = start;; --index)
    {
      indexes.push_back(index);
      if (index == end)
        break;
    }
  }
  return indexes;
}

std::vector<std::vector<std::uint32_t>> partition_path_at_waits(const ExecutePath::Goal& goal)
{
  std::vector<std::vector<std::uint32_t>> batches;
  std::vector<std::uint32_t> batch;
  for (const std::uint32_t index : execution_node_indexes(goal))
  {
    batch.push_back(index);
    if (goal.nodes.at(index - 1).wait)
    {
      batches.push_back(batch);
      batch.clear();
    }
  }
  if (!batch.empty())
    batches.push_back(std::move(batch));
  return batches;
}

}  // namespace robot_arm3_moveit_control
