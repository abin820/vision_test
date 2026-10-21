#include "set_control_enable.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace sp_nav_bt
{

static constexpr const char * DEFAULT_BB_KEY = "controller_enabled";

SetControlEnable::SetControlEnable(
  const std::string & name, const BT::NodeConfiguration & conf)
: BT::CoroActionNode(name, conf)
{
  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("SetControlEnable: rclcpp::Node::SharedPtr missing in blackboard");
  }

  std::string service_name = "/set_control_enable";
  getInput<std::string>("service_name", service_name);

  client_ = node_->create_client<std_srvs::srv::SetBool>(service_name);
}

BT::PortsList SetControlEnable::providedPorts()
{
  return {
    BT::InputPort<bool>("enable", "true = 启用控制, false = 禁用控制"),
    BT::InputPort<std::string>("service_name", "/set_control_enable",
      "SetBool 服务名称"),
    BT::InputPort<std::string>("bb_key", DEFAULT_BB_KEY,
      "记录当前使能状态的 blackboard key，相同 key 的节点共享状态"),
  };
}

BT::NodeStatus SetControlEnable::tick()
{

  bool enable;
  if (!getInput<bool>("enable", enable)) {
    RCLCPP_ERROR(node_->get_logger(), "[SetControlEnable] 缺少 'enable' 输入端口");
    return BT::NodeStatus::FAILURE;
  }

  std::string bb_key = DEFAULT_BB_KEY;
  getInput<std::string>("bb_key", bb_key);

  if (!pending_) {
    bool current_state;

    if (config().blackboard->get<bool>(bb_key, current_state) &&
        current_state == enable)
    {

      return BT::NodeStatus::SUCCESS;
    }

    if (!client_->service_is_ready()) {
      if (!client_->wait_for_service(std::chrono::milliseconds(200))) {
        RCLCPP_WARN(node_->get_logger(),
          "[SetControlEnable] 服务 '%s' 不可用",
          client_->get_service_name());
        return BT::NodeStatus::FAILURE;
      }
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    future_ = client_->async_send_request(request);
    request_start_time_ = std::chrono::steady_clock::now();
    pending_enable_ = enable;
    pending_bb_key_ = bb_key;
    pending_ = true;
  }

  while (future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() - request_start_time_ > std::chrono::seconds(2)) {
      RCLCPP_ERROR(node_->get_logger(),
        "[SetControlEnable] 服务 '%s' 请求超时，无响应",
        client_->get_service_name());
      pending_ = false;
      return BT::NodeStatus::FAILURE;
    }
    setStatusRunningAndYield();
  }

  pending_ = false;
  auto result = future_.get();

  if (!result->success) {
    RCLCPP_ERROR(node_->get_logger(),
      "[SetControlEnable] 服务返回失败: %s", result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->set<bool>(bb_key, enable);
  RCLCPP_INFO(node_->get_logger(), "[SetControlEnable] %s", result->message.c_str());

  return BT::NodeStatus::SUCCESS;
}

void SetControlEnable::halt()
{
  if (pending_) {

    config().blackboard->set<bool>(pending_bb_key_, pending_enable_);
    RCLCPP_WARN(node_->get_logger(),
      "[SetControlEnable] 等待 '%s' 响应时被中断，按已发送的 enable=%s 更新缓存",
      client_->get_service_name(), pending_enable_ ? "true" : "false");
  }
  pending_ = false;
  BT::CoroActionNode::halt();
}

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sp_nav_bt::SetControlEnable>("nav_SetControlEnable");
}