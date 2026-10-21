
#include <chrono>
#include <string>

#include "rate_control.hpp"

RateControl::RateControl(
    const std::string &name,
    const BT::NodeConfiguration &conf)
    : BT::DecoratorNode(name, conf),
      first_time_(false)
{
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
}

BT::NodeStatus RateControl::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {

    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);

  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
      seconds.count() >= period_)
  {
    first_time_ = false;
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state)
    {
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;

    case BT::NodeStatus::SUCCESS:
      start_ = std::chrono::high_resolution_clock::now();
      return BT::NodeStatus::SUCCESS;

    case BT::NodeStatus::FAILURE:
    default:
      return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<RateControl>("nav_RateControl");
}
