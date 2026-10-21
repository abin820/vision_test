#ifndef SP_DECISION_PLUGIN_GIMBAL_CONTROL_HPP_
#define SP_DECISION_PLUGIN_GIMBAL_CONTROL_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_msg/msg/gimbal_control_msg.hpp"

namespace sp_decision
{

class GimbalControl : public BT::SyncActionNode
{
public:
  GimbalControl(const std::string & name, const BT::NodeConfiguration & config);
  ~GimbalControl() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<robot_msg::msg::GimbalControlMsg>::SharedPtr pub_;
};

}

#endif
