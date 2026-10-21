#ifndef SP_DECISION_PLUGIN_CHASSIS_MODE_HPP_
#define SP_DECISION_PLUGIN_CHASSIS_MODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_msg/msg/chassis_mode_msg.hpp"

namespace sp_decision
{

class ChassisMode : public BT::SyncActionNode
{
public:
  ChassisMode(const std::string & name, const BT::NodeConfiguration & config);
  ~ChassisMode() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<robot_msg::msg::ChassisModeMsg>::SharedPtr pub_;
};

}

#endif
