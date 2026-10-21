#ifndef SP_DECISION_DECISION_ENGINE_HPP_
#define SP_DECISION_DECISION_ENGINE_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sp_decision/bt_engine.hpp"

namespace sp_decision
{

class DecisionEngine
{
public:

  explicit DecisionEngine(
    rclcpp::Node::SharedPtr node,
    std::chrono::milliseconds tick_period = std::chrono::milliseconds(50));

  ~DecisionEngine();

  void set_plugin_libraries(const std::vector<std::string> & libs);

  void set_tree_files(const std::vector<std::string> & xml_paths);

  const std::vector<std::string> & get_tree_files() const { return tree_files_; }

  bool start(const std::string & xml_file = "");

  void stop();

  bool switch_tree(const std::string & xml_file);

  bool is_running() const { return running_.load(); }
  const std::string & current_tree_file() const { return current_tree_file_; }

  BT::Blackboard::Ptr get_blackboard();

  std::function<void(const std::string &)> on_status_update;

  std::function<void()> on_tree_initialized;

private:
  void run_loop();
  bool reinit_tree(const std::string & xml_file);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<BtEngine> engine_;

  std::vector<std::string> plugin_libs_;
  std::vector<std::string> tree_files_;
  std::string current_tree_file_;

  std::thread bt_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};

  std::string pending_tree_file_;
  std::atomic<bool> switch_requested_{false};
  std::mutex switch_mutex_;

  std::chrono::milliseconds tick_period_;
};

}

#endif
