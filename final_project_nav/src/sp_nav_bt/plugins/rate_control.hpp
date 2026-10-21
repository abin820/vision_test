
#ifndef SP_DECISION_RATE_CONTROLLER_NODE
#define SP_DECISION_RATE_CONTROLLER_NODE

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

class RateControl : public BT::DecoratorNode
{
public:

  RateControl(
      const std::string &name,
      const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("hz", 10.0, "Rate")};
  }

private:

  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};

#endif
