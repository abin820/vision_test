#ifndef SP_DECISION_BT_ENGINE_HPP_
#define SP_DECISION_BT_ENGINE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "rclcpp/rclcpp.hpp"

namespace sp_decision
{

enum class BtStatus
{
  SUCCEEDED,
  FAILED,
  CANCELED,
  RUNNING
};

class BtEngine
{
public:
  explicit BtEngine(rclcpp::Node::SharedPtr node);
  virtual ~BtEngine() = default;

  void load_plugins(const std::vector<std::string> & libs);

  bool init_tree(const std::string & xml_file);

  BT::NodeStatus tick_once();

  BtStatus run(std::chrono::milliseconds loop_rate = std::chrono::milliseconds(100));

  void halt();

  BT::Blackboard::Ptr get_blackboard() { return blackboard_; }

protected:
  rclcpp::Node::SharedPtr node_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree                tree_;
  BT::Blackboard::Ptr     blackboard_;

  bool is_halted_ = false;
};

}

#endif
