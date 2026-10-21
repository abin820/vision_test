#ifndef SP_DECISION_PLUGIN_NAV_TO_POSE_HPP_
#define SP_DECISION_PLUGIN_NAV_TO_POSE_HPP_

#include <atomic>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msg/action/navigate_to_pose.hpp"

namespace BT
{
template <>
inline geometry_msgs::msg::PoseStamped
convertFromString<geometry_msgs::msg::PoseStamped>(StringView str)
{

    auto stripped = str;
    while (!stripped.empty() &&
           (stripped.front() == '(' || stripped.front() == ' '))
        stripped.remove_prefix(1);
    while (!stripped.empty() &&
           (stripped.back() == ')' || stripped.back() == ' '))
        stripped.remove_suffix(1);

    std::vector<double> v;
    std::string token;
    for (char c : stripped)
    {
        if (c == ',') {
            v.push_back(std::stod(token));
            token.clear();
        } else {
            token += c;
        }
    }
    if (!token.empty())
        v.push_back(std::stod(token));

    if (v.size() != 2 && v.size() != 3 && v.size() != 7)
        throw RuntimeError(
            "convertFromString<PoseStamped>: expected 2, 3, or 7 values, got " +
            std::to_string(v.size()));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id   = "map";
    pose.pose.position.x   = v[0];
    pose.pose.position.y   = v[1];
    pose.pose.position.z   = (v.size() == 7) ? v[2] : 0.0;

    if (v.size() == 7) {

        pose.pose.orientation.x = v[3];
        pose.pose.orientation.y = v[4];
        pose.pose.orientation.z = v[5];
        pose.pose.orientation.w = v[6];
    } else if (v.size() == 3) {

        const double half_yaw = v[2] * 0.5;
        pose.pose.orientation.w = std::cos(half_yaw);
        pose.pose.orientation.z = std::sin(half_yaw);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
    } else {

        pose.pose.orientation.w = 1.0;
    }

    return pose;
}
}

namespace sp_decision
{

class NavToPose : public BT::StatefulActionNode
{
public:
  using NavigateToPose = robot_msg::action::NavigateToPose;
  using GoalHandle     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavToPose(const std::string & name, const BT::NodeConfiguration & config);

  ~NavToPose() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:

  enum class ActionState : int
  {
    IDLE      = 0,
    PENDING   = 1,
    RUNNING   = 2,
    SUCCEEDED = 3,
    FAILED    = 4,
    CANCELED  = 5,
  };

  rclcpp::Node::SharedPtr                            node_;
  rclcpp::executors::SingleThreadedExecutor          executor_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr   action_client_;

  GoalHandle::SharedPtr   goal_handle_;
  std::atomic<int>        action_state_{static_cast<int>(ActionState::IDLE)};
  std::atomic<uint64_t>   goal_gen_{0};

  void goal_response_cb(const GoalHandle::SharedPtr & goal_handle);

  void feedback_cb(
    GoalHandle::SharedPtr ,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void result_cb(const GoalHandle::WrappedResult & result);

  static std::optional<geometry_msgs::msg::PoseStamped>
    parse_goal_string(const std::string & s);};

}

#endif
