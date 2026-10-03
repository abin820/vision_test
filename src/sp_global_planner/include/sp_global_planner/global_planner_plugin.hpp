#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace sp_global_planner
{

  // 接口，比较重要，具体实现在Astar
  class GlobalPlannerPlugin
  {
  public:
    virtual ~GlobalPlannerPlugin() = default;

    virtual void configure(
        const rclcpp::Node::SharedPtr &node,
        const std::string &plugin_name) = 0;

    virtual void setMap(const nav_msgs::msg::OccupancyGrid &costmap) = 0;

    virtual nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) = 0;
  };

}
