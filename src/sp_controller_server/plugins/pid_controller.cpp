#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

// 读参数，若 yaml 里没有则用 fallback 默认值
template <typename T>
T declare_param(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & fallback)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<T>();
  }
  node->declare_parameter<T>(name, fallback);
  return node->get_parameter(name).get_value<T>();
}

}

namespace pid_controller {

void PidController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;
  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }
  base_frame_id_ = require_param<std::string>(node_, plugin_name_ + ".base_frame_id");

  // —— 全向底盘位置追踪参数 ——
  kp_linear_           = declare_param<double>(node_, plugin_name_ + ".kp_linear", 1.0);
  max_linear_velocity_ = declare_param<double>(node_, plugin_name_ + ".max_linear_velocity", 0.5);
  lookahead_distance_  = declare_param<double>(node_, plugin_name_ + ".lookahead_distance", 0.5);
  goal_tolerance_      = declare_param<double>(node_, plugin_name_ + ".goal_tolerance", 0.15);

  RCLCPP_INFO(node_->get_logger(),
              "[%s] 全向位置追踪 configured | kp_linear=%.2f v_max=%.2f "
              "lookahead=%.2f goal_tol=%.2f",
              plugin_name_.c_str(),
              kp_linear_, max_linear_velocity_,
              lookahead_distance_, goal_tolerance_);
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  (void)velocity;  // 全向底盘：只追踪位置，不需要速度/朝向反馈

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  // 没有路径（或只有起点）→ 原地停车
  if (global_plan_.poses.size() < 2) {
    return cmd_vel;
  }

  const double cur_x = pose.pose.position.x;
  const double cur_y = pose.pose.position.y;

  // 1) 路径上离车最近的点
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double d = std::hypot(global_plan_.poses[i].pose.position.x - cur_x,
                                global_plan_.poses[i].pose.position.y - cur_y);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }

  // 2) 从最近点沿路径前进 lookahead_distance_ 米，取前瞻点
  size_t lookahead_idx = global_plan_.poses.size() - 1;  // 默认终点兜底
  double accumulated = 0.0;
  for (size_t i = closest_idx; i + 1 < global_plan_.poses.size(); ++i) {
    const double dx = global_plan_.poses[i + 1].pose.position.x - global_plan_.poses[i].pose.position.x;
    const double dy = global_plan_.poses[i + 1].pose.position.y - global_plan_.poses[i].pose.position.y;
    accumulated += std::hypot(dx, dy);
    if (accumulated >= lookahead_distance_) {
      lookahead_idx = i + 1;
      break;
    }
  }

  const double target_x = global_plan_.poses[lookahead_idx].pose.position.x;
  const double target_y = global_plan_.poses[lookahead_idx].pose.position.y;

  // 3) 到终点距离：用于停车 + 减速
  const double goal_x = global_plan_.poses.back().pose.position.x;
  const double goal_y = global_plan_.poses.back().pose.position.y;
  const double dist_to_goal = std::hypot(goal_x - cur_x, goal_y - cur_y);

  if (dist_to_goal < goal_tolerance_) {
    return cmd_vel;  // 到终点附近 → 停车
  }

  // 4) 位置误差 → 速度方向（全向底盘：直接朝目标点走，无需朝向对齐）
  const double dx = target_x - cur_x;
  const double dy = target_y - cur_y;
  const double dist = std::hypot(dx, dy);

  // 5) 速度大小：随剩余距离线性衰减，接近终点减速
  const double v = std::clamp(kp_linear_ * dist_to_goal, 0.0, max_linear_velocity_);

  // 6) 全向合成：base_link 已锁定(与 map 系重合 yaw=0)，世界系速度即车体系速度
  cmd_vel.twist.linear.x = (dist > 1e-6) ? v * dx / dist : 0.0;
  cmd_vel.twist.linear.y = (dist > 1e-6) ? v * dy / dist : 0.0;
  cmd_vel.twist.angular.z = 0.0;
  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
