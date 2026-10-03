#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "sp_controller_server/controller_plugin.hpp"

namespace pid_controller {

class PidController : public sp_controller_server::ControllerPlugin {
public:
  PidController() = default;
  ~PidController() override = default;

  void configure(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

private:
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path global_plan_;
  std::string base_frame_id_;

  // —— 速度环（控制速度大小，反馈 odom 实际速度）——
  double kp_linear_{1.0};   // 参考速度增益：目标速度 = kp_linear × 剩余距离
  double kp_v_{0.0};        // 速度环 P（跟踪目标速度）
  double ki_v_{0.0};        // 速度环 I
  double kd_v_{0.0};        // 速度环 D
  double max_linear_velocity_{0.5};

  // —— 方向环（控制运动方向，反馈当前朝向）——
  double kp_theta_{1.0};    // 方向环 P（1.0 = 纯追踪）
  double ki_theta_{0.0};    // 方向环 I（建议 0）
  double kd_theta_{0.0};    // 方向环 D

  // —— 路径跟踪 ——
  double lookahead_distance_{0.5};
  double goal_tolerance_{0.15};

  // —— PID 状态（跨调用保存）——
  double integral_v_{0.0};
  double prev_error_v_{0.0};
  double integral_theta_{0.0};
  double prev_error_theta_{0.0};
  rclcpp::Time last_time_;
  bool pid_initialized_{false};
};

}

#endif
