#pragma once

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace nav3d_util
{

class NodeThread
{
public:
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  template<typename NodeT>
  explicit NodeThread(const NodeT & node)
  : NodeThread(node->get_node_base_interface())
  {}

  ~NodeThread();

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
};

}  // namespace nav3d_util
