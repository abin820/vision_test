#ifndef SP_DECISION_BLACKBOARD_BRIDGE_HPP_
#define SP_DECISION_BLACKBOARD_BRIDGE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sp_decision
{

  class BlackboardBridge : public rclcpp::Node
  {
  public:
    explicit BlackboardBridge(
        BT::Blackboard::Ptr blackboard,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~BlackboardBridge() override = default;

    void set_blackboard(BT::Blackboard::Ptr blackboard);

  private:
    void init_all_variables();
    void on_goal_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void on_clicked_point(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    template <typename T>
    void write(const std::string &key, const T &value)
    {
      std::lock_guard<std::mutex> lk(bb_mutex_);
      if (blackboard_)
      {
        blackboard_->set<T>(key, value);
      }
    }

    BT::Blackboard::Ptr blackboard_;
    std::mutex bb_mutex_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_clicked_point_;
  };

}

#endif
