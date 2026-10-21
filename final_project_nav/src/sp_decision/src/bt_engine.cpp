#include "sp_decision/bt_engine.hpp"

namespace sp_decision
{

BtEngine::BtEngine(rclcpp::Node::SharedPtr node)
: node_(node)
{
  blackboard_ = BT::Blackboard::create();

  blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
}

void BtEngine::load_plugins(const std::vector<std::string> & libs)
{
  BT::SharedLibrary loader;
  for (const auto & lib : libs)
  {
    RCLCPP_INFO(node_->get_logger(), "[BtEngine] Loading plugin: %s", lib.c_str());
    try {
      factory_.registerFromPlugin(loader.getOSName(lib));
    } catch (const std::exception & e1) {
      RCLCPP_WARN(node_->get_logger(),
        "[BtEngine] OSName failed, trying literal: %s", lib.c_str());
      try {
        factory_.registerFromPlugin(lib);
      } catch (const std::exception & e2) {
        RCLCPP_ERROR(node_->get_logger(),
          "[BtEngine] Could not load plugin %s: %s", lib.c_str(), e2.what());
      }
    }
  }
}

bool BtEngine::init_tree(const std::string & xml_file)
{
  RCLCPP_INFO(node_->get_logger(),
    "[BtEngine] Initialising tree from: %s", xml_file.c_str());
  try
  {
    tree_ = factory_.createTreeFromFile(xml_file, blackboard_);
    return true;
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[BtEngine] Failed to initialise tree: %s", ex.what());
    return false;
  }
}

BT::NodeStatus BtEngine::tick_once()
{
  is_halted_ = false;
  return tree_.tickRoot();
}

BtStatus BtEngine::run(std::chrono::milliseconds loop_rate_ms)
{
  rclcpp::WallRate rate(loop_rate_ms);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  is_halted_ = false;

  tree_.rootNode()->halt();

  RCLCPP_INFO(node_->get_logger(), "[BtEngine] Running Behavior Tree …");

  try
  {
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && !is_halted_)
    {
      status = tree_.tickRoot();
      rate.sleep();
    }
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(node_->get_logger(),
      "[BtEngine] Exception during run: %s", ex.what());
    return BtStatus::FAILED;
  }

  if (is_halted_)                              { return BtStatus::CANCELED; }
  if (status == BT::NodeStatus::SUCCESS)       { return BtStatus::SUCCEEDED; }
  return BtStatus::FAILED;
}

void BtEngine::halt()
{
  is_halted_ = true;
  if (tree_.rootNode()) {
    tree_.rootNode()->halt();
  }
}

}
