#pragma once

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace robot_arm3_moveit_control
{

enum class FlyQueueType
{
  NONE,
  CARTESIAN,
  JOINT,
};

enum class FlySegmentType
{
  LINEAR,
  CIRCULAR,
  JOINT,
};

struct FlySegment
{
  FlySegmentType type;
  std::vector<double> values;
};

enum class FlyMode
{
  NORMAL,
  CARTESIAN,
};

struct FlySettings
{
  FlyMode mode{ FlyMode::NORMAL };
  double normal_percent{ 75.0 };
  double stress_percent{ 10.0 };
  int trajectory_mode{ 0 };
  double distance_mm{ 5.0 };
};

class FlyQueueError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class FlyQueue
{
public:
  explicit FlyQueue(std::size_t max_points);

  void add(FlySegmentType type, const std::vector<double>& values);
  void clear();

  std::size_t size() const;
  bool empty() const;
  FlyQueueType type() const;
  const std::vector<FlySegment>& segments() const;

private:
  std::size_t max_points_;
  FlyQueueType type_{ FlyQueueType::NONE };
  std::vector<FlySegment> segments_;
};

}  // namespace robot_arm3_moveit_control
