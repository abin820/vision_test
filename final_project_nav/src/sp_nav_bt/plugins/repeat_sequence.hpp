

#ifndef SP_DECISION_REPEAT_SEQUENCE_NODE
#define SP_DECISION_REPEAT_SEQUENCE_NODE

#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "repeat_sequence_halt_hook.hpp"

class RepeatSequence : public BT::ControlNode
{
public:

  explicit RepeatSequence(const std::string & name);

  RepeatSequence(const std::string & name, const BT::NodeConfiguration & config);

  void halt() override;

  static BT::PortsList providedPorts() {return {};}

protected:

  BT::NodeStatus tick() override;

  std::size_t last_child_ticked_ = 0;
};

#endif
