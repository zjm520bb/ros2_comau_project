#include <gtest/gtest.h>

#include "robot_arm3_moveit_control/path_model.hpp"

namespace control = robot_arm3_moveit_control;
using PathNode = arm_tcp_bridge_interfaces::msg::PathNode;

namespace
{

PathNode node(std::uint8_t type, bool wait = false)
{
  PathNode value;
  value.motion_type = type;
  value.target = { 1, 2, 3, 4, 5, 6 };
  value.linear_speed = 0.05;
  value.rotational_speed = 10.0;
  value.segment_override = 20.0;
  value.termination_type = 1;
  value.tolerance = 1.0;
  value.segment_data = true;
  value.fly = false;
  value.fly_type = 0;
  value.fly_percent = 75.0;
  value.fly_distance_mm = 5.0;
  value.fly_trajectory = 0;
  value.stress_percent = 10.0;
  value.wait = wait;
  return value;
}

control::ExecutePath::Goal goal()
{
  control::ExecutePath::Goal value;
  value.path_id = 1;
  value.path_type = control::ExecutePath::Goal::CARTESIAN;
  value.nodes = { node(PathNode::LINEAR), node(PathNode::SEG_VIA), node(PathNode::CIRCULAR) };
  return value;
}

}  // namespace

TEST(PathModel, ValidatesCircularStructure)
{
  EXPECT_NO_THROW(control::validate_path_goal(goal(), 20));
  auto invalid = goal();
  invalid.nodes = { node(PathNode::CIRCULAR) };
  EXPECT_THROW(control::validate_path_goal(invalid, 20), control::PathModelError);
}

TEST(PathModel, PartitionsAtWaitNodes)
{
  auto value = goal();
  value.nodes = { node(PathNode::LINEAR, true), node(PathNode::LINEAR) };
  control::validate_path_goal(value, 20);
  const auto batches = control::partition_path_at_waits(value);
  ASSERT_EQ(batches.size(), 2u);
  EXPECT_EQ(batches[0], std::vector<std::uint32_t>({ 1 }));
  EXPECT_EQ(batches[1], std::vector<std::uint32_t>({ 2 }));
}

TEST(PathModel, SupportsReverseLinearRange)
{
  auto value = goal();
  value.nodes = { node(PathNode::LINEAR), node(PathNode::LINEAR), node(PathNode::LINEAR) };
  value.start_index = 3;
  value.end_index = 1;
  control::validate_path_goal(value, 20);
  EXPECT_EQ(control::execution_node_indexes(value),
            std::vector<std::uint32_t>({ 3, 2, 1 }));
}
