#include "astar_planner.hpp"
#include <queue>
#include <limits>
#include <cmath>
#include <algorithm>
#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>

namespace
{

  [[noreturn]] void sp_nav_param_error(const rclcpp::Node &node, const std::string &name)
  {
    std::ostringstream oss;
    oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
        << "'，请在对应 yaml 的 ros__parameters 中填写。";
    throw std::runtime_error(oss.str());
  }

  template <typename T>
  T require_param(const rclcpp::Node::SharedPtr &node, const std::string &name)
  {
    try
    {
      if (node->has_parameter(name))
      {
        return node->get_parameter(name).get_value<T>();
      }
      return node->declare_parameter<T>(name);
    }
    catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &)
    {
      sp_nav_param_error(*node, name);
    }
  }

  template <typename T>
  T declare_param(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &fallback)
  {
    if (node->has_parameter(name))
    {
      return node->get_parameter(name).get_value<T>();
    }
    node->declare_parameter<T>(name, fallback);
    return node->get_parameter(name).get_value<T>();
  }

}

namespace sp_global_planner
{

  void AStarPlanner::configure(const rclcpp::Node::SharedPtr &node, const std::string &plugin_name)
  {
    logger_ = node->get_logger();

    lethal_cost_ = require_param<int>(node, plugin_name + ".lethal_cost");
    cost_weight_ = require_param<double>(node, plugin_name + ".cost_weight");

    optimize_path_     = declare_param<bool>(node, plugin_name + ".optimize_path", true);
    opt_alpha_         = declare_param<double>(node, plugin_name + ".opt_alpha", 0.2);
    opt_beta_          = declare_param<double>(node, plugin_name + ".opt_beta", 0.4);
    opt_gamma_         = declare_param<double>(node, plugin_name + ".opt_gamma", 0.5);
    opt_safe_distance_ = declare_param<double>(node, plugin_name + ".opt_safe_distance", 0.6);
    opt_min_clearance_ = declare_param<double>(node, plugin_name + ".opt_min_clearance", 0.3);
    opt_iterations_    = declare_param<int>(node, plugin_name + ".opt_iterations", 50);

    RCLCPP_INFO(logger_,
                "AStarPlanner configured: lethal_cost=%d cost_weight=%.3f optimize=%d "
                "alpha=%.2f beta=%.2f gamma=%.2f safe=%.2f iter=%d",
                lethal_cost_, cost_weight_, optimize_path_ ? 1 : 0,
                opt_alpha_, opt_beta_, opt_gamma_, opt_safe_distance_, opt_iterations_);
  }

  void AStarPlanner::setMap(const nav_msgs::msg::OccupancyGrid &costmap)
  {
    map_ = costmap;
  }

  void AStarPlanner::setEsdf(const nav_msgs::msg::OccupancyGrid &esdf)
  {
    esdf_map_ = esdf;
  }

  bool AStarPlanner::isBlocked(int8_t c) const // 判断是否是障碍物
  {

    if (c < 0)
      return true;
    return c >= lethal_cost_;
  }

  double AStarPlanner::cellCostFactor(int8_t c) const // 计算代价因子
  {

    double cc = std::max<int>(0, c);
    return 1.0 + cost_weight_ * (cc / 100.0);
  }

  nav_msgs::msg::Path AStarPlanner::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    nav_msgs::msg::Path path;
    if (!map_)
    {
      RCLCPP_ERROR(logger_, "No map received yet.");
      return path;
    }
    const auto &map = *map_;
    path.header = map.header;

    if (start.header.frame_id != map.header.frame_id || goal.header.frame_id != map.header.frame_id)
    {
      RCLCPP_WARN(logger_, "Frame mismatch: start=%s goal=%s map=%s",
                  start.header.frame_id.c_str(), goal.header.frame_id.c_str(), map.header.frame_id.c_str());
    }

    GridIndex s, g;
    if (!worldToGrid(map, start.pose.position.x, start.pose.position.y, s))
    {
      RCLCPP_ERROR(logger_, "Start out of map bounds.");
      return path;
    }
    if (!worldToGrid(map, goal.pose.position.x, goal.pose.position.y, g))
    {
      RCLCPP_ERROR(logger_, "Goal out of map bounds.");
      return path;
    }

    const int W = static_cast<int>(map.info.width);
    const int H = static_cast<int>(map.info.height);
    const int N = W * H;

    auto idx = [&](int x, int y)
    { return y * W + x; };

    if (isBlocked(map.data[idx(s.x, s.y)]))
    {
      RCLCPP_ERROR(logger_, "Start is in blocked cell (cost=%d).", (int)map.data[idx(s.x, s.y)]);
      return path;
    }
    if (isBlocked(map.data[idx(g.x, g.y)]))
    {
      RCLCPP_ERROR(logger_, "Goal is in blocked cell (cost=%d).", (int)map.data[idx(g.x, g.y)]);
      return path;
    }

    struct Node
    {
      int i;
      double f;
      double g;
    };
    struct Cmp
    {
      bool operator()(const Node &a, const Node &b) const { return a.f > b.f; }
    };

    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    std::vector<double> gscore(N, std::numeric_limits<double>::infinity());
    std::vector<int> parent(N, -1);
    std::vector<uint8_t> closed(N, 0);

    auto h = [&](int x, int y)
    {
      double dx = (x - g.x);
      double dy = (y - g.y);
      return std::sqrt(dx * dx + dy * dy);
    };

    int s_i = idx(s.x, s.y);
    int g_i = idx(g.x, g.y);

    gscore[s_i] = 0.0;
    open.push({s_i, h(s.x, s.y), 0.0});

    const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    bool found = false;

    while (!open.empty())
    {
      Node cur = open.top();
      open.pop();

      if (closed[cur.i])
        continue;
      closed[cur.i] = 1;

      if (cur.i == g_i)
      {
        found = true;
        break;
      }

      int cy = cur.i / W;
      int cx = cur.i - cy * W;

      for (int k = 0; k < 8; ++k)
      {
        int nx = cx + dxs[k];
        int ny = cy + dys[k];
        if (!inBounds(map, nx, ny))
          continue;

        int ni = idx(nx, ny);
        if (closed[ni])
          continue;

        int8_t c = map.data[ni];
        if (isBlocked(c))
          continue;

        double step = (k < 4) ? 1.0 : std::sqrt(2.0);

        double factor = cellCostFactor(c);
        double tentative = gscore[cur.i] + step * factor;

        if (tentative < gscore[ni])
        {
          gscore[ni] = tentative;
          parent[ni] = cur.i;
          double f = tentative + h(nx, ny);
          open.push({ni, f, tentative});
        }
      }
    }

    if (!found)
    {
      RCLCPP_WARN(logger_, "A* failed to find a path.");
      return path;
    }

    std::vector<int> cells;
    int cur = g_i;
    while (cur != -1)
    {
      cells.push_back(cur);
      if (cur == s_i)
        break;
      cur = parent[cur];
    }
    if (cells.back() != s_i)
    {
      RCLCPP_WARN(logger_, "Path reconstruction failed.");
      return path;
    }
    std::reverse(cells.begin(), cells.end());

    path.poses.reserve(cells.size());
    for (int ci : cells)
    {
      int y = ci / W;
      int x = ci - y * W;

      double wx, wy;
      gridToWorld(map, x, y, wx, wy);

      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = wx;
      ps.pose.position.y = wy;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }

    // 路径优化：把贴墙的路径拉向走廊中心并平滑（基于 ESDF 距离场梯度下降）
    if (optimize_path_) {
      path = optimizePath(path);
    }

    return path; // A*算法f(n) = g(n) + h(n),后续可以看看h(n)有没有优化的办法
  }

  // 查询距离场在 (wx, wy) 处的值（双线性插值，返回米；>0 表示离最近障碍物的距离）
  double AStarPlanner::esdfDistance(
    const nav_msgs::msg::OccupancyGrid &esdf, double wx, double wy) const
  {
    const auto &info = esdf.info;
    const double res = info.resolution;
    const double ox = info.origin.position.x;
    const double oy = info.origin.position.y;
    const int W = static_cast<int>(info.width);
    const int H = static_cast<int>(info.height);

    const double gx = (wx - ox) / res;
    const double gy = (wy - oy) / res;
    const int x0 = static_cast<int>(std::floor(gx));
    const int y0 = static_cast<int>(std::floor(gy));
    const double fx = gx - static_cast<double>(x0);
    const double fy = gy - static_cast<double>(y0);

    auto sample = [&](int x, int y) -> double {
      x = std::clamp(x, 0, W - 1);
      y = std::clamp(y, 0, H - 1);
      return static_cast<double>(esdf.data[y * W + x]) / 100.0;  // 值/100 = 米
    };

    const double v00 = sample(x0, y0);
    const double v10 = sample(x0 + 1, y0);
    const double v01 = sample(x0, y0 + 1);
    const double v11 = sample(x0 + 1, y0 + 1);
    return (v00 * (1.0 - fx) + v10 * fx) * (1.0 - fy) +
           (v01 * (1.0 - fx) + v11 * fx) * fy;
  }

  // 距离场梯度（中心差分）：指向距离增大的方向，即远离墙、朝向走廊中心
  std::pair<double, double> AStarPlanner::esdfGradient(
    const nav_msgs::msg::OccupancyGrid &esdf, double wx, double wy) const
  {
    const double h = static_cast<double>(esdf.info.resolution);
    const double gx = (esdfDistance(esdf, wx + h, wy) - esdfDistance(esdf, wx - h, wy)) / (2.0 * h);
    const double gy = (esdfDistance(esdf, wx, wy + h) - esdfDistance(esdf, wx, wy - h)) / (2.0 * h);
    return {gx, gy};
  }

  // 三项目标梯度下降：
  //   平滑项(alpha)：拉向左右邻点中点，消除锯齿
  //   障碍项(beta)： 距墙过近时沿距离场梯度推开（越贴墙越强）
  //   参考项(gamma)：拉回原始 A* 路径，防止切角/漂移
  nav_msgs::msg::Path AStarPlanner::optimizePath(const nav_msgs::msg::Path &path) const
  {
    if (!esdf_map_ || path.poses.size() < 3) {
      return path;
    }
    const auto &esdf = *esdf_map_;

    struct Pt { double x, y; };
    std::vector<Pt> pts;
    pts.reserve(path.poses.size());
    for (const auto &p : path.poses) {
      pts.push_back({p.pose.position.x, p.pose.position.y});
    }
    const std::vector<Pt> ref = pts;  // 原始 A* 路径作为参考

    for (int iter = 0; iter < opt_iterations_; ++iter) {
      for (size_t i = 1; i + 1 < pts.size(); ++i) {  // 起点终点不动
        // 平滑项
        const double gx_s = pts[i - 1].x - 2.0 * pts[i].x + pts[i + 1].x;
        const double gy_s = pts[i - 1].y - 2.0 * pts[i].y + pts[i + 1].y;

        // 参考项
        const double gx_r = ref[i].x - pts[i].x;
        const double gy_r = ref[i].y - pts[i].y;

        // 障碍项
        double gx_o = 0.0, gy_o = 0.0;
        const double d = esdfDistance(esdf, pts[i].x, pts[i].y);
        if (d < opt_safe_distance_) {
          const auto grad = esdfGradient(esdf, pts[i].x, pts[i].y);
          const double w = opt_safe_distance_ - d;  // 越贴墙权重越大
          gx_o = grad.first * w;
          gy_o = grad.second * w;
        }

        const double nx = pts[i].x + opt_alpha_ * gx_s + opt_beta_ * gx_o + opt_gamma_ * gx_r;
        const double ny = pts[i].y + opt_alpha_ * gy_s + opt_beta_ * gy_o + opt_gamma_ * gy_r;
        // 安全约束：新位置必须离墙足够远，否则保持原样（防止优化把点推进墙里）
        if (esdfDistance(esdf, nx, ny) > opt_min_clearance_) {
          pts[i].x = nx;
          pts[i].y = ny;
        }
      }
    }

    nav_msgs::msg::Path out = path;
    for (size_t i = 0; i < pts.size(); ++i) {
      out.poses[i].pose.position.x = pts[i].x;
      out.poses[i].pose.position.y = pts[i].y;
    }
    return out;
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sp_global_planner::AStarPlanner, sp_global_planner::GlobalPlannerPlugin)
