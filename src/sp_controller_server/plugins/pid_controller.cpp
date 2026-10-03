#include "pid_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

constexpr double kPi = 3.14159265358979323846;

// 把角度归一到 [-π, π]
double normalize_angle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
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

  // 速度环
  kp_linear_           = declare_param<double>(node_, plugin_name_ + ".kp_linear", 1.0);
  kp_v_                = declare_param<double>(node_, plugin_name_ + ".kp_v", 0.0);
  ki_v_                = declare_param<double>(node_, plugin_name_ + ".ki_v", 0.0);
  kd_v_                = declare_param<double>(node_, plugin_name_ + ".kd_v", 0.0);
  max_linear_velocity_ = declare_param<double>(node_, plugin_name_ + ".max_linear_velocity", 0.5);
  // 方向环
  kp_theta_            = declare_param<double>(node_, plugin_name_ + ".kp_theta", 1.0);
  ki_theta_            = declare_param<double>(node_, plugin_name_ + ".ki_theta", 0.0);
  kd_theta_            = declare_param<double>(node_, plugin_name_ + ".kd_theta", 0.0);
  // 路径跟踪
  lookahead_distance_  = declare_param<double>(node_, plugin_name_ + ".lookahead_distance", 0.5);
  goal_tolerance_      = declare_param<double>(node_, plugin_name_ + ".goal_tolerance", 0.15);

  RCLCPP_INFO(node_->get_logger(),
              "[%s] 双环PID configured | 速度环: kp_v=%.2f ki_v=%.2f kd_v=%.2f "
              "参考增益=%.2f v_max=%.2f | 方向环: kp_theta=%.2f ki_theta=%.2f kd_theta=%.2f | "
              "lookahead=%.2f goal_tol=%.2f",
              plugin_name_.c_str(),
              kp_v_, ki_v_, kd_v_, kp_linear_, max_linear_velocity_,
              kp_theta_, ki_theta_, kd_theta_,
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
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  // 没有路径（或只有起点）→ 原地停车
  if (global_plan_.poses.size() < 2) {
    return cmd_vel;
  }

  // 时间步长 dt（控制频率 50Hz，约 0.02s），首次调用初始化
  const rclcpp::Time now = node_->now();
  if (!pid_initialized_) {
    last_time_ = now;
    pid_initialized_ = true;
  }
  double dt = (now - last_time_).seconds();
  last_time_ = now;
  dt = std::clamp(dt, 1e-3, 0.1);

  const double cur_x = pose.pose.position.x;
  const double cur_y = pose.pose.position.y;

  // 当前朝向（从四元数取出 yaw）
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  const double cur_yaw = tf2::getYaw(q);

  // 1) 找路径上离机器人最近的点
  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double dx = global_plan_.poses[i].pose.position.x - cur_x;
    const double dy = global_plan_.poses[i].pose.position.y - cur_y;
    const double d = std::hypot(dx, dy);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }

  // 2) 从最近点往前累计 lookahead_distance_ 米，找到前瞻点
  size_t lookahead_idx = global_plan_.poses.size() - 1;  // 默认用终点兜底
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

  // 3) 目标方向（指向前瞻点）+ 到终点距离
  const double target_x = global_plan_.poses[lookahead_idx].pose.position.x;
  const double target_y = global_plan_.poses[lookahead_idx].pose.position.y;
  const double target_yaw = std::atan2(target_y - cur_y, target_x - cur_x);

  const double goal_x = global_plan_.poses.back().pose.position.x;
  const double goal_y = global_plan_.poses.back().pose.position.y;
  const double dist_to_goal = std::hypot(goal_x - cur_x, goal_y - cur_y);

  // 4) 到终点附近 → 停车，并清空积分项（避免下次启动带历史误差）
  if (dist_to_goal < goal_tolerance_) {
    integral_v_ = prev_error_v_ = 0.0;
    integral_theta_ = prev_error_theta_ = 0.0;
    return cmd_vel;
  }

  // ===== 速度环 PID（反馈 odom 实际速度大小 |velocity|）=====
  // 目标速度由剩余距离生成（离终点越远越快），实际速度用里程计反馈
  const double target_v = std::clamp(kp_linear_ * dist_to_goal, 0.0, max_linear_velocity_);
  const double actual_v = std::hypot(velocity.linear.x, velocity.linear.y);
  const double e_v = target_v - actual_v;
  integral_v_ += e_v * dt;
  integral_v_ = std::clamp(integral_v_, -max_linear_velocity_, max_linear_velocity_);  // 抗积分饱和
  const double d_v = (e_v - prev_error_v_) / dt;
  prev_error_v_ = e_v;
  // 前馈 target_v + PID 修正：稳态时误差趋 0，速度收敛到目标速度
  double v_cmd = target_v + kp_v_ * e_v + ki_v_ * integral_v_ + kd_v_ * d_v;
  v_cmd = std::clamp(v_cmd, 0.0, max_linear_velocity_);

  // ===== 方向环 PID（反馈当前朝向 cur_yaw）=====
  // 误差 = 目标方向 − 当前朝向（归一 [-π, π]），输出 body 系速度方向角
  const double e_theta = normalize_angle(target_yaw - cur_yaw);
  integral_theta_ += e_theta * dt;
  integral_theta_ = std::clamp(integral_theta_, -1.0, 1.0);  // 抗积分饱和
  const double d_theta = (e_theta - prev_error_theta_) / dt;
  prev_error_theta_ = e_theta;
  double theta_cmd = kp_theta_ * e_theta + ki_theta_ * integral_theta_ + kd_theta_ * d_theta;
  theta_cmd = normalize_angle(theta_cmd);

  // ===== 全向底盘合成（angular.z 仿真器忽略）=====
  cmd_vel.twist.linear.x = v_cmd * std::cos(theta_cmd);
  cmd_vel.twist.linear.y = v_cmd * std::sin(theta_cmd);
  cmd_vel.twist.angular.z = 0.0;
  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
