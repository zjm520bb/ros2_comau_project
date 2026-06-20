#include "robot_arm3_moveit_control/fly_queue.hpp"

namespace robot_arm3_moveit_control
{

FlyQueue::FlyQueue(std::size_t max_points) : max_points_(max_points)
{
  if (max_points_ == 0)
    throw FlyQueueError("max_fly_points must be greater than zero");
}

void FlyQueue::add(FlySegmentType type, const std::vector<double>& values)
{
  if (segments_.size() >= max_points_)
    throw FlyQueueError("FLY queue is full");

  const FlyQueueType requested_type =
      type == FlySegmentType::JOINT ? FlyQueueType::JOINT : FlyQueueType::CARTESIAN;
  if (type_ != FlyQueueType::NONE && type_ != requested_type)
    throw FlyQueueError("Cannot mix JOINT and CARTESIAN segments in one FLY queue");

  type_ = requested_type;
  segments_.push_back({ type, values });
}

void FlyQueue::clear()
{
  segments_.clear();
  type_ = FlyQueueType::NONE;
}

std::size_t FlyQueue::size() const
{
  return segments_.size();
}

bool FlyQueue::empty() const
{
  return segments_.empty();
}

FlyQueueType FlyQueue::type() const
{
  return type_;
}

const std::vector<FlySegment>& FlyQueue::segments() const
{
  return segments_;
}

}  // namespace robot_arm3_moveit_control
