#pragma once
#include "sp_global_planner/global_planner_plugin.hpp"
#include "sp_global_planner/grid_utils.hpp"
#include <optional>
#include <utility>
#include <vector>

namespace sp_global_planner
{

  class AStarPlanner : public GlobalPlannerPlugin
  {
  public:
    AStarPlanner() = default;
    ~AStarPlanner() override = default;

    void configure(const rclcpp::Node::SharedPtr &node, const std::string &plugin_name) override;
    void setMap(const nav_msgs::msg::OccupancyGrid &costmap) override;

    void setEsdf(const nav_msgs::msg::OccupancyGrid &esdf) override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override;

  private:
    std::optional<nav_msgs::msg::OccupancyGrid> map_;
    std::optional<nav_msgs::msg::OccupancyGrid> esdf_map_;  // 距离场，值/100 = 米
    rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};

    int lethal_cost_{100};
    double cost_weight_{2.0};

    // 路径优化参数
    bool optimize_path_{true};
    double opt_alpha_{0.2};
    double opt_beta_{0.4};
    double opt_gamma_{0.5};
    double opt_safe_distance_{0.6};
    double opt_min_clearance_{0.3};  // 安全约束：优化时点离墙的最小距离 (m)
    int opt_iterations_{50};

    bool isBlocked(int8_t c) const;
    double cellCostFactor(int8_t c) const;

    // 路径优化（ESDF 梯度下降）
    double esdfDistance(const nav_msgs::msg::OccupancyGrid &esdf, double wx, double wy) const;
    std::pair<double, double> esdfGradient(const nav_msgs::msg::OccupancyGrid &esdf, double wx, double wy) const;
    nav_msgs::msg::Path optimizePath(const nav_msgs::msg::Path &path) const;
  };

}
