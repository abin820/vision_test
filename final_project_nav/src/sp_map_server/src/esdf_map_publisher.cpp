#include <rclcpp/rclcpp.hpp>
#include <rclcpp/exceptions.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

namespace
{

  [[noreturn]] void sp_nav_param_error(const rclcpp::Node &node, const std::string &name)
  {
    std::ostringstream oss;
    oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
        << "'，请在对应 yaml 的 ros__parameters 中填写。";
    throw std::runtime_error(oss.str());
  }

  template <typename T> // 后面在类里面使用
  T require_param(rclcpp::Node *node, const std::string &name)
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

  std::string resolve_package_uri(const std::string &raw) // 解析package://路径,看不懂啥意思
  {
    if (raw.empty() || raw.front() == '/')
    {
      return raw;
    }
    std::string rest = raw;
    const std::string prefix = "package://";
    if (raw.compare(0, prefix.size(), prefix) == 0)
    {
      rest = raw.substr(prefix.size());
    }
    const auto slash = rest.find('/');
    if (slash == std::string::npos)
    {
      return raw;
    }
    const std::string pkg = rest.substr(0, slash);
    const std::string rel = rest.substr(slash + 1);
    return (fs::path(ament_index_cpp::get_package_share_directory(pkg)) / rel).string();
  }

}

static std::string join_path_if_relative(const std::string &base_dir, const std::string &maybe_rel) // 这个也看不懂
{
  if (maybe_rel.empty())
    return maybe_rel;
  fs::path p(maybe_rel);
  if (p.is_absolute())
    return maybe_rel;
  return (fs::path(base_dir) / p).string();
}

static cv::Mat load_pgm_grayscale_u8(const std::string &image_path)
{
  cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (img.empty())
  {
    throw std::runtime_error("Failed to read image: " + image_path);
  }
  if (img.type() != CV_8UC1)
  {
    img.convertTo(img, CV_8UC1);
  }
  return img;
}

static cv::Mat distance_transform_meters_from_binary_u8(const cv::Mat &binary_u8, float resolution)
{

  cv::Mat dist;
  cv::distanceTransform(binary_u8, dist, cv::DIST_L2, 5);
  dist *= resolution;
  return dist;
}
// 上面有很多函数先不管了，这篇代码的作用就是根据yaml文件生成esdf_map，能够快速判断机器人是否会和障碍物碰撞
class EsdfMapPublisher : public rclcpp::Node
{
public:
  EsdfMapPublisher() : Node("esdf_map_publisher")
  {
    map_yaml_path_ = resolve_package_uri(require_param<std::string>(this, "map_yaml"));
    if (map_yaml_path_.empty())
    {
      sp_nav_param_error(*this, "map_yaml");
    }
    frame_id_ = require_param<std::string>(this, "frame_id");
    require_param<double>(this, "publish_rate_hz");
    require_param<bool>(this, "unknown_as_obstacle");
    require_param<double>(this, "robot_radius");
    require_param<double>(this, "margin");
    require_param<double>(this, "d_safe");
    require_param<double>(this, "w");
    require_param<int>(this, "unknown_cost");
    require_param<bool>(this, "visualize_esdf");

    esdf_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/esdf", 1);
    global_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_costmap", 1);
    esdf_costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/esdf_costmap", 1);

    load_and_compute();

    double rate = this->get_parameter("publish_rate_hz").as_double();
    double period = 1.0 / std::max(rate, 0.1);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&EsdfMapPublisher::on_timer, this));
  }

private:
  void load_and_compute()
  {
    YAML::Node cfg = YAML::LoadFile(map_yaml_path_);

    if (!cfg["image"] || !cfg["resolution"] || !cfg["origin"])
    {
      throw std::runtime_error("map yaml missing required keys: image/resolution/origin");
    }

    std::string image_rel = cfg["image"].as<std::string>();
    double resolution = cfg["resolution"].as<double>();
    YAML::Node origin = cfg["origin"];

    int negate = cfg["negate"] ? cfg["negate"].as<int>() : 0;
    double occ_thresh = cfg["occupied_thresh"] ? cfg["occupied_thresh"].as<double>() : 0.65;
    double free_thresh = cfg["free_thresh"] ? cfg["free_thresh"].as<double>() : 0.196;

    std::string yaml_dir = fs::path(map_yaml_path_).parent_path().string();
    std::string image_path = join_path_if_relative(yaml_dir, image_rel);

    cv::Mat pgm = load_pgm_grayscale_u8(image_path);
    const int H = pgm.rows;
    const int W = pgm.cols;

    cv::Mat pgm_f;
    pgm.convertTo(pgm_f, CV_32FC1, 1.0 / 255.0);

    cv::Mat occ_prob = (negate == 0) ? (1.0f - pgm_f) : pgm_f;

    cv::Mat occ_img(H, W, CV_16SC1, cv::Scalar(-1));
    for (int y = 0; y < H; ++y)
    {
      const float *row = occ_prob.ptr<float>(y);
      int16_t *out = occ_img.ptr<int16_t>(y);
      for (int x = 0; x < W; ++x)
      {
        float p = row[x];
        if (p >= occ_thresh)
          out[x] = 100;
        else if (p <= free_thresh)
          out[x] = 0;
        else
          out[x] = -1;
      }
    }

    bool unknown_as_obstacle = this->get_parameter("unknown_as_obstacle").as_bool();

    cv::Mat obstacle_img(H, W, CV_8UC1, cv::Scalar(0));
    cv::Mat free_img(H, W, CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < H; ++y)
    {
      const int16_t *row = occ_img.ptr<int16_t>(y);
      uint8_t *ob = obstacle_img.ptr<uint8_t>(y);
      uint8_t *fr = free_img.ptr<uint8_t>(y);
      for (int x = 0; x < W; ++x)
      {
        bool is_occ = (row[x] == 100);
        bool is_unk = (row[x] == -1);
        bool is_obs = is_occ || (unknown_as_obstacle && is_unk);

        bool is_free = (row[x] == 0) || (!unknown_as_obstacle && is_unk);

        ob[x] = is_obs ? 255 : 0;
        fr[x] = (!is_obs && is_free) ? 255 : 0;
      }
    }

    cv::Mat dist_free_to_obs = distance_transform_meters_from_binary_u8(free_img, static_cast<float>(resolution));

    cv::Mat obstacle_space(H, W, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
    {
      const uint8_t *fr = free_img.ptr<uint8_t>(y);
      uint8_t *os = obstacle_space.ptr<uint8_t>(y);
      for (int x = 0; x < W; ++x)
      {
        os[x] = (fr[x] == 0) ? 255 : 0;
      }
    }
    cv::Mat dist_obs_to_free = distance_transform_meters_from_binary_u8(obstacle_space, static_cast<float>(resolution));

    esdf_.assign(H * W, 0.0f);
    for (int y = 0; y < H; ++y)
    {
      const uint8_t *ob = obstacle_img.ptr<uint8_t>(y);
      const float *dfo = dist_free_to_obs.ptr<float>(y);
      const float *dof = dist_obs_to_free.ptr<float>(y);
      int gy = (H - 1 - y);
      for (int x = 0; x < W; ++x)
      {
        int idx = gy * W + x;
        esdf_[idx] = (ob[x] != 0) ? -dof[x] : dfo[x];
      }
    }

    std_msgs::msg::Float32MultiArray esdf_msg;
    esdf_msg.layout.dim.resize(2);
    esdf_msg.layout.dim[0].label = "height";
    esdf_msg.layout.dim[0].size = H;
    esdf_msg.layout.dim[0].stride = H * W;
    esdf_msg.layout.dim[1].label = "width";
    esdf_msg.layout.dim[1].size = W;
    esdf_msg.layout.dim[1].stride = W;
    esdf_msg.data.assign(esdf_.begin(), esdf_.end());
    esdf_msg_ = esdf_msg;

    double robot_radius = this->get_parameter("robot_radius").as_double();
    double margin = this->get_parameter("margin").as_double();
    double d_safe = this->get_parameter("d_safe").as_double();
    double w_pen = this->get_parameter("w").as_double();
    int unknown_cost = this->get_parameter("unknown_cost").as_int();

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width = W;
    grid.info.height = H;

    grid.info.origin.position.x = origin[0].as<double>();
    grid.info.origin.position.y = origin[1].as<double>();
    double yaw = origin[2].as<double>();
    grid.info.origin.orientation.z = std::sin(yaw / 2.0);
    grid.info.origin.orientation.w = std::cos(yaw / 2.0);

    grid.data.resize(H * W);

    for (int i = 0; i < H * W; ++i)
    {
      float d = esdf_[i];

      float clearance = d - static_cast<float>(robot_radius + margin);
      if (clearance < 0.0f)
      {
        grid.data[i] = 100;
      }
      else
      {
        if (d >= static_cast<float>(d_safe))
        {
          grid.data[i] = 0;
        }
        else
        {
          float diff = static_cast<float>(d_safe) - d;
          float penalty = static_cast<float>(w_pen) * diff * diff;
          float scaled = 100.0f * (1.0f - std::exp(-penalty));
          int c = static_cast<int>(std::round(std::min(99.0f, std::max(0.0f, scaled))));
          grid.data[i] = static_cast<int8_t>(c);
        }
      }
    }
    costmap_msg_ = grid;

    nav_msgs::msg::OccupancyGrid esdf_grid = grid;
    for (int i = 0; i < H * W; ++i)
    {
      float d = esdf_[i];

      int8_t c = static_cast<int8_t>(std::min(127.0f, std::max(-128.0f, d * 100.0f)));
      esdf_grid.data[i] = c;
    }
    esdf_costmap_msg_ = esdf_grid;

    visualize_esdf_ = this->get_parameter("visualize_esdf").as_bool();
    if (visualize_esdf_)
    {
      cv::Mat esdf_vis(H, W, CV_8UC3);
      for (int y = 0; y < H; ++y)
      {
        for (int x = 0; x < W; ++x)
        {

          float d = esdf_[(H - 1 - y) * W + x];
          if (d <= 0)
          {
            esdf_vis.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
          }
          else
          {

            uint8_t val = static_cast<uint8_t>(std::min(255.0f, d * 127.0f));
            esdf_vis.at<cv::Vec3b>(y, x) = cv::Vec3b(val, val, val);
          }
        }
      }
      cv::applyColorMap(esdf_vis, color_map_, cv::COLORMAP_JET);

      for (int y = 0; y < H; ++y)
      {
        for (int x = 0; x < W; ++x)
        {
          if (esdf_[(H - 1 - y) * W + x] <= 0)
          {
            color_map_.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
          }
        }
      }
    }
    if (visualize_esdf_)
    {
      cv::imshow("ESDF Visualization", color_map_);
      cv::waitKey(1);
    }
    RCLCPP_INFO(this->get_logger(),
                "Loaded map: %s  size=%dx%d  res=%.3f m/cell  origin=[%.3f, %.3f, %.3f]  negate=%d  unknown_as_obstacle=%s",
                image_path.c_str(), W, H, resolution,
                origin[0].as<double>(), origin[1].as<double>(), origin[2].as<double>(),
                negate, unknown_as_obstacle ? "true" : "false");
  }

  void on_timer()
  {

    esdf_pub_->publish(esdf_msg_);

    costmap_msg_.header.stamp = this->now();
    global_costmap_pub_->publish(costmap_msg_);

    esdf_costmap_msg_.header.stamp = this->now();
    esdf_costmap_pub_->publish(esdf_costmap_msg_);
  }

private:
  std::string map_yaml_path_;
  std::string frame_id_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr esdf_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<float> esdf_;
  bool visualize_esdf_ = false;
  cv::Mat color_map_;
  std_msgs::msg::Float32MultiArray esdf_msg_;
  nav_msgs::msg::OccupancyGrid costmap_msg_;
  nav_msgs::msg::OccupancyGrid esdf_costmap_msg_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<EsdfMapPublisher>(); // 创建esdf_map发布者
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    std::cerr << "Fatal: " << e.what() << std::endl; // 输出异常信息
  }
  rclcpp::shutdown();
  return 0;
}
