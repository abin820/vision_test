#include "condition_judge.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wextra"
#include "../third_part/exprtk.hpp"
#pragma GCC diagnostic pop

#include "behaviortree_cpp_v3/bt_factory.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace sp_decision
{

struct ConditionJudge::ExprImpl
{
  exprtk::symbol_table<double>             symbol_table;
  std::vector<exprtk::expression<double>>  expressions;
  exprtk::parser<double>                   parser;
};

ConditionJudge::ConditionJudge(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
, impl_(std::make_unique<ExprImpl>())
{}

ConditionJudge::~ConditionJudge() = default;

BT::PortsList ConditionJudge::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "bb_vars",
      "Semicolon-separated blackboard key names exposed to expressions as "
      "doubles, e.g. \"hp;game_status;enemy_detected\""),
    BT::InputPort<std::string>(
      "conditions",
      "Semicolon-separated exprtk expressions, one per child.  "
      "First truthy expression selects the child.  "
      "Raw operators <, >, &&, || may be used; XML entities are sanitised "
      "automatically, e.g. \"game_status<1; game_status==1 && hp>60; true\""),
  };
}

std::string ConditionJudge::sanitize_expr(const std::string & raw)
{

  std::string s = raw;
  auto replace_all = [](std::string & str,
                         const std::string & from,
                         const std::string & to) {
    std::size_t pos = 0;
    while ((pos = str.find(from, pos)) != std::string::npos) {
      str.replace(pos, from.size(), to);
      pos += to.size();
    }
  };

  replace_all(s, "&amp;&amp;", "&&");
  replace_all(s, "&amp;",     "&");
  replace_all(s, "&lt;",      "<");
  replace_all(s, "&gt;",      ">");
  replace_all(s, "&apos;",    "'");
  replace_all(s, "&quot;",    "\"");

  replace_all(s, "&&", " and ");
  replace_all(s, "||", " or ");
  return s;
}

std::vector<std::string> ConditionJudge::split(
  const std::string & s, char delim)
{
  std::vector<std::string> tokens;
  std::stringstream ss(s);
  std::string token;
  while (std::getline(ss, token, delim)) {

    const auto start = token.find_first_not_of(" \t\r\n");
    const auto end   = token.find_last_not_of(" \t\r\n");
    if (start != std::string::npos) {
      tokens.push_back(token.substr(start, end - start + 1));
    } else {
      tokens.push_back("");
    }
  }
  return tokens;
}

double ConditionJudge::read_bb_double(const std::string & key) const
{
  auto bb = config().blackboard;
  if (!bb) { return 0.0; }

  try { return bb->get<double>(key);   } catch (...) {}
  try { return static_cast<double>(bb->get<float>(key));   } catch (...) {}
  try { return static_cast<double>(bb->get<int32_t>(key)); } catch (...) {}
  try { return static_cast<double>(bb->get<int>(key));     } catch (...) {}
  try { return static_cast<double>(bb->get<bool>(key));    } catch (...) {}
  try { return std::stod(bb->get<std::string>(key));       } catch (...) {}
  return 0.0;
}

void ConditionJudge::sync_vars()
{
  for (const auto & name : var_names_) {
    var_vals_[name] = read_bb_double(name);

    impl_->symbol_table.get_variable(name)->ref() = var_vals_[name];
  }
}

bool ConditionJudge::init_expressions()
{

  std::string bb_vars_str;
  if (!getInput<std::string>("bb_vars", bb_vars_str)) {
    RCLCPP_ERROR(rclcpp::get_logger("ConditionJudge"),
      "[ConditionJudge] Missing required port 'bb_vars'");
    return false;
  }
  var_names_ = split(bb_vars_str, ';');

  std::string cond_str;
  if (!getInput<std::string>("conditions", cond_str)) {
    RCLCPP_ERROR(rclcpp::get_logger("ConditionJudge"),
      "[ConditionJudge] Missing required port 'conditions'");
    return false;
  }
  const auto raw_conds = split(cond_str, ';');

  if (raw_conds.size() != childrenCount()) {
    RCLCPP_ERROR(rclcpp::get_logger("ConditionJudge"),
      "[ConditionJudge] conditions count (%zu) != children count (%zu)",
      raw_conds.size(), childrenCount());
    return false;
  }

  impl_->symbol_table.add_constants();
  for (const auto & name : var_names_) {
    var_vals_[name] = 0.0;
    impl_->symbol_table.add_variable(name, var_vals_[name]);
  }

  impl_->expressions.resize(raw_conds.size());
  for (std::size_t i = 0; i < raw_conds.size(); ++i) {
    const std::string expr_str = sanitize_expr(raw_conds[i]);
    impl_->expressions[i].register_symbol_table(impl_->symbol_table);
    if (!impl_->parser.compile(expr_str, impl_->expressions[i])) {
      RCLCPP_ERROR(rclcpp::get_logger("ConditionJudge"),
        "[ConditionJudge] Failed to compile expression[%zu] \"%s\": %s",
        i, expr_str.c_str(),
        impl_->parser.error().c_str());
      return false;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("ConditionJudge"),
      "[ConditionJudge] Compiled expression[%zu]: \"%s\"", i, expr_str.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("ConditionJudge"),
    "[ConditionJudge] Initialised: %zu variables, %zu conditions, %zu children.",
    var_names_.size(), raw_conds.size(), childrenCount());
  return true;
}

BT::NodeStatus ConditionJudge::tick()
{

  if (!initialized_) {
    if (!init_expressions()) {
      return BT::NodeStatus::FAILURE;
    }
    initialized_ = true;
  }

  sync_vars();

  int new_idx = -1;
  for (std::size_t i = 0; i < impl_->expressions.size(); ++i) {
    if (impl_->expressions[i].value() != 0.0) {
      new_idx = static_cast<int>(i);
      break;
    }
  }

  if (new_idx < 0) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("ConditionJudge"),
      *rclcpp::Clock::make_shared(), 2000,
      "[ConditionJudge] No condition matched – returning FAILURE.");

    if (current_child_idx_ >= 0) {
      children()[current_child_idx_]->halt();
      current_child_idx_ = -1;
    }
    return BT::NodeStatus::FAILURE;
  }

  if (new_idx != current_child_idx_) {
    if (current_child_idx_ >= 0 &&
        children()[current_child_idx_]->status() == BT::NodeStatus::RUNNING)
    {
      RCLCPP_INFO(rclcpp::get_logger("ConditionJudge"),
        "[ConditionJudge] Switching child %d -> %d, halting previous.",
        current_child_idx_, new_idx);
      children()[current_child_idx_]->halt();
    }
    current_child_idx_ = new_idx;
  }

  auto * child = children()[current_child_idx_];
  const auto child_status = child->status();
  if (child_status != BT::NodeStatus::RUNNING &&
      child_status != BT::NodeStatus::IDLE)
  {
    child->halt();
  }

  const BT::NodeStatus status = child->executeTick();

  if (status != BT::NodeStatus::RUNNING) {
    current_child_idx_ = -1;
  }

  return status;
}

void ConditionJudge::halt()
{
  if (current_child_idx_ >= 0 &&
      current_child_idx_ < static_cast<int>(childrenCount()))
  {
    if (children()[current_child_idx_]->status() == BT::NodeStatus::RUNNING) {
      children()[current_child_idx_]->halt();
    }
  }
  current_child_idx_ = -1;
  setStatus(BT::NodeStatus::IDLE);
}

}

extern "C" __attribute__((visibility("default")))
void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<sp_decision::ConditionJudge>("ConditionJudge");
}
