#ifndef SP_NAV_BT_NAV_INTERFACE_NODE_HPP_
#define SP_NAV_BT_NAV_INTERFACE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msg/action/navigate_to_pose.hpp"
#include "sp_nav_bt/nav_bt_engine.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace sp_nav_bt
{

class NavInterfaceNode : public rclcpp::Node
{
public:
  using NavigateToPose = robot_msg::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit NavInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~NavInterfaceNode() = default;

  void init_engine();

protected:

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNav> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle);

  void execute_tree();

  void handle_restart_task(
    const std_srvs::srv::Trigger::Request::SharedPtr  req,
    const std_srvs::srv::Trigger::Response::SharedPtr res);

  std::shared_ptr<NavBTEngine> bt_engine_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_to_pose_server_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_task_srv_;

  std::mutex bt_mutex_;
  std::atomic<bool> is_task_running_{false};

  bool is_restarting_{false};
  std::shared_ptr<GoalHandleNav> current_goal_handle_;

  geometry_msgs::msg::PoseStamped original_goal_pose_;
  bool has_original_goal_pose_{false};

  std::string default_bt_xml_;
  std::vector<std::string> plugin_libs_;

  std::thread bt_thread_;
};

}

#endif
