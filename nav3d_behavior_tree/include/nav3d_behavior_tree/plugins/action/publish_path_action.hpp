#pragma once

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>

namespace nav3d_behavior_tree {

class PublishPath : public BT::ActionNodeBase {
  public:
    PublishPath(const std::string& name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        return {BT::InputPort<nav_msgs::msg::Path>("path", "Path to publish"),
                BT::InputPort<std::string>("topic_name", std::string("remaining_plan"), "Topic name"),
                BT::InputPort<int>("queue_size", 1, "Publisher queue size")};
    }

    BT::NodeStatus tick() override;
    void halt() override {}

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    std::string topic_name_{"remaining_plan"};
    int queue_size_{1};

    void ensurePublisher();
};

} // namespace nav3d_behavior_tree