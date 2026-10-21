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

  // —— 全向底盘位置追踪：速度大小随剩余距离衰减，方向直接指向前瞻点 ——
  double kp_linear_{1.0};           // 速度增益 v = kp_linear × 剩余距离
  double max_linear_velocity_{0.5}; // 速度上限 (m/s)
  double lookahead_distance_{0.5};  // 前瞻距离 (m)
  double goal_tolerance_{0.15};     // 到达容差 (m)
};

}

#endif
