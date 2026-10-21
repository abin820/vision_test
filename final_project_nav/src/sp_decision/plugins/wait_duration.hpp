#ifndef SP_DECISION_PLUGIN_WAIT_DURATION_HPP_
#define SP_DECISION_PLUGIN_WAIT_DURATION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sp_decision
{

class WaitDuration : public BT::StatefulActionNode
{
public:
  WaitDuration(const std::string & name, const BT::NodeConfiguration & config);
  ~WaitDuration() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  rclcpp::Node::SharedPtr node_;

  bool  infinite_{false};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::duration<double>         target_duration_{0};
};

}

#endif
