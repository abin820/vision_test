#ifndef SP_NAV_BT_PLUGINS_NAV_STATE_TRACK_HPP_
#define SP_NAV_BT_PLUGINS_NAV_STATE_TRACK_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sp_nav_bt
{

class NavStateTrack : public BT::StatefulActionNode
{
public:
  NavStateTrack(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::chrono::steady_clock::time_point start_time_;
};

}

#endif
