#include "sp_nav_bt/nav_bt_engine.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace sp_nav_bt
{

NavBTEngine::NavBTEngine(rclcpp::Node::SharedPtr node)
: node_(node)
{
  blackboard_ = BT::Blackboard::create();

  blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);
}

void NavBTEngine::load_plugins(const std::vector<std::string> &plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto &lib_name : plugin_libraries)
  {
    RCLCPP_INFO(node_->get_logger(), "Loading plugin: %s", lib_name.c_str());
    try
    {

      factory_.registerFromPlugin(loader.getOSName(lib_name));
    }
    catch (const std::exception &ex)
    {
       RCLCPP_WARN(node_->get_logger(), "Failed to load via OSName, trying literal: %s", lib_name.c_str());
       try {

         factory_.registerFromPlugin(lib_name);
       } catch (const std::exception &ex2) {
         RCLCPP_ERROR(node_->get_logger(), "Failed to load BT plugin %s: %s", lib_name.c_str(), ex2.what());
       }
    }
  }
}

bool NavBTEngine::init_tree(const std::string &xml_file)
{
  RCLCPP_INFO(node_->get_logger(), "Initializing BT from: %s", xml_file.c_str());
  try
  {
    tree_ = factory_.createTreeFromFile(xml_file, blackboard_);

    if (node_->get_parameter("enable_bt_cout_logger").as_bool()) {
      std_cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
      RCLCPP_INFO(node_->get_logger(), "BT StdCoutLogger enabled.");
    }

    return true;
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "BehaviorTree initialization error: %s", ex.what());
    return false;
  }
}

BtStatus NavBTEngine::run(std::chrono::milliseconds loop_rate_ms)
{
  rclcpp::WallRate loop_rate(loop_rate_ms);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  is_halted_ = false;

  tree_.rootNode()->halt();

  RCLCPP_INFO(node_->get_logger(), "Running Behavior Tree...");

  try
  {
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && !is_halted_)
    {
      status = tree_.tickRoot();

      if (!loop_rate.sleep())
      {

      }
    }
  }
  catch (const std::exception &ex)
  {
      RCLCPP_ERROR(node_->get_logger(), "BT execution exception: %s", ex.what());
      return BtStatus::FAILED;
  }

  if (is_halted_) return BtStatus::CANCELED;
  return (status == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

void NavBTEngine::halt()
{
  is_halted_ = true;
  tree_.rootNode()->halt();
}

}
