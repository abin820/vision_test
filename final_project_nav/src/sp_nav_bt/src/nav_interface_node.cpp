#include "sp_nav_bt/nav_interface_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(rclcpp::Node * node, const std::string & name)
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

std::string resolve_package_uri(const std::string & raw)
{
  if (raw.empty() || raw.front() == '/') {
    return raw;
  }
  std::string rest = raw;
  const std::string prefix = "package://";
  if (raw.compare(0, prefix.size(), prefix) == 0) {
    rest = raw.substr(prefix.size());
  }
  const auto slash = rest.find('/');
  if (slash == std::string::npos) {
    return raw;
  }
  const std::string pkg = rest.substr(0, slash);
  const std::string rel = rest.substr(slash + 1);
  return (std::filesystem::path(ament_index_cpp::get_package_share_directory(pkg)) / rel).string();
}

}

namespace sp_nav_bt
{

NavInterfaceNode::NavInterfaceNode(const rclcpp::NodeOptions & options)
: Node("nav_interface_node", options)
{

  default_bt_xml_ = resolve_package_uri(require_param<std::string>(this, "default_bt_xml"));
  plugin_libs_ = require_param<std::vector<std::string>>(this, "bt_plugins");
  require_param<bool>(this, "enable_bt_cout_logger");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void NavInterfaceNode::init_engine()
{

  bt_engine_ = std::make_shared<NavBTEngine>(this->shared_from_this());
  bt_engine_->load_plugins(plugin_libs_);

  if (!bt_engine_->init_tree(default_bt_xml_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initial static BT: %s", default_bt_xml_.c_str());
  }

  this->nav_to_pose_server_ = rclcpp_action::create_server<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "navigate_to_pose",
    std::bind(&NavInterfaceNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavInterfaceNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&NavInterfaceNode::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Navigation Interface Node initialized.");

  restart_task_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/nav_interface/restart_task",
    std::bind(&NavInterfaceNode::handle_restart_task, this,
              std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(),
    "Restart-task service ready at /nav_interface/restart_task");
}

rclcpp_action::GoalResponse NavInterfaceNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request: Go to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavInterfaceNode::handle_cancel(
  const std::shared_ptr<GoalHandleNav> goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(this->get_logger(), "Received cancel request");

  std::lock_guard<std::mutex> lock(bt_mutex_);
  if (is_task_running_) {
    bt_engine_->halt();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavInterfaceNode::handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
{

  std::unique_lock<std::mutex> lock(bt_mutex_);
  if (is_task_running_) {
    RCLCPP_INFO(this->get_logger(), "Preempting previous task.");
    bt_engine_->halt();

    lock.unlock();
    if (bt_thread_.joinable()) {
      bt_thread_.join();
    }
    lock.lock();
  }

  current_goal_handle_ = goal_handle;

  original_goal_pose_ = goal_handle->get_goal()->pose;
  has_original_goal_pose_ = true;
  is_task_running_ = true;

  if (bt_thread_.joinable()) {
    lock.unlock();
    bt_thread_.join();
    lock.lock();
  }

  bt_thread_ = std::thread(&NavInterfaceNode::execute_tree, this);
}

void NavInterfaceNode::execute_tree()
{
  auto result = std::make_shared<NavigateToPose::Result>();

  try {

    auto blackboard = bt_engine_->get_blackboard();

    blackboard->set<bool>("controller_enabled", false);

    geometry_msgs::msg::PoseStamped goal_pose;
    {
      std::lock_guard<std::mutex> lock(bt_mutex_);
      if (has_original_goal_pose_) {
        goal_pose = original_goal_pose_;
      } else if (current_goal_handle_) {
        goal_pose = current_goal_handle_->get_goal()->pose;
        original_goal_pose_ = goal_pose;
        has_original_goal_pose_ = true;
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "execute_tree: no original goal available, aborting.");
        is_task_running_ = false;
        return;
      }
    }

    blackboard->set<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose);
    RCLCPP_INFO(this->get_logger(),
      "BT goal_pose set to ORIGINAL action goal: (%.3f, %.3f)",
      goal_pose.pose.position.x, goal_pose.pose.position.y);

    blackboard->set<std::string>("task_id", "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(), "Executing Static BT for goal...");
    BtStatus status = bt_engine_->run(std::chrono::milliseconds(10));

    {
      std::lock_guard<std::mutex> lock(bt_mutex_);
      is_task_running_ = false;

      if (is_restarting_) {
        RCLCPP_INFO(this->get_logger(), "BT halted for restart, skipping goal finalization.");
        return;
      }

      if (status == BtStatus::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Navigation task finished successfully.");
        current_goal_handle_->succeed(result);
      } else if (status == BtStatus::CANCELED) {
        if (current_goal_handle_->is_canceling()) {
          RCLCPP_WARN(this->get_logger(), "Navigation task canceled.");
          current_goal_handle_->canceled(result);
        } else {
          RCLCPP_WARN(this->get_logger(), "Navigation task preempted or halted without cancel request.");
          current_goal_handle_->abort(result);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Navigation task failed.");
        current_goal_handle_->abort(result);
      }

      blackboard->set<std::string>("task_id", "idle");
      has_original_goal_pose_ = false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in execute_tree: %s", e.what());
    std::lock_guard<std::mutex> lock(bt_mutex_);
    is_task_running_ = false;

  }
}

}

namespace sp_nav_bt
{

void NavInterfaceNode::handle_restart_task(
  const std_srvs::srv::Trigger::Request::SharedPtr  ,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::unique_lock<std::mutex> lock(bt_mutex_);

  if (!is_task_running_ || !current_goal_handle_ || !has_original_goal_pose_) {
    res->success = false;
    res->message = "No active navigation task (or original goal) to restart";
    RCLCPP_WARN(this->get_logger(),
      "[restart_task] No active task / original goal, ignoring restart request.");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "[restart_task] Relocalization done — halt BT, restart with ORIGINAL goal (%.3f, %.3f)",
    original_goal_pose_.pose.position.x, original_goal_pose_.pose.position.y);

  is_restarting_ = true;
  bt_engine_->halt();

  lock.unlock();
  if (bt_thread_.joinable()) {
    bt_thread_.join();
  }
  lock.lock();

  is_restarting_   = false;
  is_task_running_ = true;
  bt_thread_ = std::thread(&NavInterfaceNode::execute_tree, this);

  res->success = true;
  res->message = "BT restarted with original action goal";
  RCLCPP_INFO(this->get_logger(),
    "[restart_task] BT thread restarted with original goal (%.3f, %.3f).",
    original_goal_pose_.pose.position.x, original_goal_pose_.pose.position.y);
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sp_nav_bt::NavInterfaceNode>();
  node->init_engine();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
