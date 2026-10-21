#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <QApplication>
#include <QFont>
#include <QFontDatabase>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#include "sp_decision/blackboard_bridge.hpp"
#include "sp_decision/decision_engine.hpp"
#include "sp_decision/decision_gui.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
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

class DecisionNode : public rclcpp::Node
{
public:
    DecisionNode()
        : rclcpp::Node("sp_decision")
    {
        require_param<std::vector<std::string>>(this, "tree_files");
        require_param<std::vector<std::string>>(this, "bt_plugins");
        require_param<double>(this, "tick_rate_hz");
    }
};

static void setup_chinese_font(QApplication & app)
{
  static const char * candidates[] = {
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "WenQuanYi Micro Hei",
    "WenQuanYi Zen Hei",
    "Source Han Sans SC",
    "Microsoft YaHei",
    "PingFang SC",
    "SimHei",
    nullptr,
  };

  const QStringList families = QFontDatabase().families();
  QFont font = app.font();
  for (int i = 0; candidates[i] != nullptr; ++i) {
    if (families.contains(QString::fromUtf8(candidates[i]))) {
      font.setFamily(QString::fromUtf8(candidates[i]));
      app.setFont(font);
      return;
    }
  }
}

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);
    app.setApplicationName("sp_decision");
    app.setOrganizationName("SentinelProject");
    setup_chinese_font(app);

    rclcpp::init(argc, argv);

    auto decision_node = std::make_shared<DecisionNode>();

    const auto tree_files_raw =
      decision_node->get_parameter("tree_files").as_string_array();
    std::vector<std::string> tree_files;
    tree_files.reserve(tree_files_raw.size());
    for (const auto & path : tree_files_raw) {
      tree_files.push_back(resolve_package_uri(path));
    }
    const auto bt_plugins =
        decision_node->get_parameter("bt_plugins").as_string_array();
    const double tick_hz =
        decision_node->get_parameter("tick_rate_hz").as_double();

    const auto tick_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(tick_hz, 1.0)));

    auto engine = std::make_shared<sp_decision::DecisionEngine>(
        decision_node, tick_period);

    engine->set_plugin_libraries(bt_plugins);
    engine->set_tree_files(tree_files);

    auto bridge_node = std::make_shared<sp_decision::BlackboardBridge>(
        nullptr );

    engine->on_tree_initialized = [&bridge_node, &engine]()
    {
        auto bb = engine->get_blackboard();
        if (bb)
        {
            bridge_node->set_blackboard(bb);
        }
    };

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(decision_node);
    executor.add_node(bridge_node);

    std::atomic<bool> ros_ok{true};
    std::thread ros_thread([&executor, &ros_ok]()
                           {
    executor.spin();
    ros_ok.store(false); });

    sp_decision::DecisionGui gui(engine);
    gui.show();

    if (!tree_files.empty()) {
        engine->start(tree_files.front());
    }

    const int qt_ret = app.exec();

    engine->stop();
    executor.cancel();
    rclcpp::shutdown();
    if (ros_thread.joinable())
    {
        ros_thread.join();
    }

    return qt_ret;
}
