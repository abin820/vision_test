#ifndef SP_NAV_BT_PLUGINS_SET_CONTROL_ENABLE_HPP_
#define SP_NAV_BT_PLUGINS_SET_CONTROL_ENABLE_HPP_

#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace sp_nav_bt
{

class SetControlEnable : public BT::CoroActionNode
{
public:
  SetControlEnable(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_;
  bool pending_{false};
  bool pending_enable_{false};
  std::string pending_bb_key_;
  std::chrono::steady_clock::time_point request_start_time_;
};

}

#endif