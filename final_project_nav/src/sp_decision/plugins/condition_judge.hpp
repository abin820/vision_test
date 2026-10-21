#ifndef SP_DECISION_PLUGIN_CONDITION_JUDGE_HPP_
#define SP_DECISION_PLUGIN_CONDITION_JUDGE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviortree_cpp_v3/control_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sp_decision
{

class ConditionJudge : public BT::ControlNode
{
public:
  explicit ConditionJudge(
    const std::string & name,
    const BT::NodeConfiguration & config);

  ~ConditionJudge() override;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
  void           halt() override;

private:

  bool initialized_{false};

  bool init_expressions();

  struct ExprImpl;
  std::unique_ptr<ExprImpl> impl_;

  std::vector<std::string>           var_names_;
  std::unordered_map<std::string, double> var_vals_;

  void sync_vars();

  double read_bb_double(const std::string & key) const;

  int current_child_idx_{-1};

  static std::string sanitize_expr(const std::string & raw);

  static std::vector<std::string> split(const std::string & s, char delim);
};

}

#endif
