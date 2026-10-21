#include "gimbal_control.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace sp_decision
{

GimbalControl::GimbalControl(
    const std::string & name,
    const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    pub_ = node_->create_publisher<robot_msg::msg::GimbalControlMsg>(
        "/gimbal/control", rclcpp::QoS(1).reliable());

    RCLCPP_INFO(node_->get_logger(), "[GimbalControl] BT node initialised.");
}

BT::PortsList GimbalControl::providedPorts()
{
    return {
        BT::InputPort<int>  ("mode",            0,     "0=全向扫描, 1=范围扫描, 2=yaw锁定, 3=yaw指定角度"),
        BT::InputPort<float>("big_yaw",         0.0f,  "yaw 目标角度 (deg)，mode=3 时生效"),
        BT::InputPort<float>("yaw_lower_limit", 0.0f,  "yaw 范围下限 (deg)，mode=1 时生效"),
        BT::InputPort<float>("yaw_upper_limit", 0.0f,  "yaw 范围上限 (deg)，mode=1 时生效"),
    };
}

BT::NodeStatus GimbalControl::tick()
{
    robot_msg::msg::GimbalControlMsg msg;

    int mode_int = 0;
    getInput("mode",            mode_int);
    msg.mode = static_cast<uint8_t>(mode_int);
    getInput("big_yaw",         msg.big_yaw);
    getInput("yaw_lower_limit", msg.yaw_lower_limit);
    getInput("yaw_upper_limit", msg.yaw_upper_limit);

    pub_->publish(msg);

    RCLCPP_DEBUG(node_->get_logger(),
                 "[GimbalControl] Published mode=%u  big_yaw=%.1f",
                 msg.mode, msg.big_yaw);

    return BT::NodeStatus::SUCCESS;
}

}

extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
    factory.registerNodeType<sp_decision::GimbalControl>("GimbalControl");
}
