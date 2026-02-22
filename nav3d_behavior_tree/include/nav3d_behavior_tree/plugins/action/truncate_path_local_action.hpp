#pragma once

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace nav3d_behavior_tree {

class TruncatePathLocal : public BT::ActionNodeBase {
  public:
    TruncatePathLocal(const std::string& name, const BT::NodeConfiguration& conf);

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<nav_msgs::msg::Path>("input_path", "Path to truncate"),
            BT::OutputPort<nav_msgs::msg::Path>("output_path", "Truncated path output"),

            BT::InputPort<double>("distance_forward", 0.0, "Distance forward along the path (m)"),
            BT::InputPort<double>("distance_backward", 0.0, "Distance backward along the path (m)"),

            BT::InputPort<std::string>("robot_frame", "Robot base frame id"),
            BT::InputPort<double>("transform_tolerance", 0.0, "Transform lookup tolerance (s)"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Override pose instead of TF"),

            BT::InputPort<double>("angular_distance_weight", 0.0, "Weight for angular distance in closest pose search"),
            BT::InputPort<double>("max_robot_pose_search_dist", std::numeric_limits<double>::infinity(),
                                  "Max integrated distance ahead to search closest pose (m)"),

            BT::InputPort<std::string>("mode", "local", "Truncation mode: local or remaining"),
            BT::InputPort<double>("resample_distance", 0.0, "If > 0, resample output path by spacing (m)"),
            BT::InputPort<int>("max_output_poses", 0, "If > 0, cap output path size"),
        };
    }

    BT::NodeStatus tick() override;
    void halt() override {}

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    nav_msgs::msg::Path path_;
    std::vector<geometry_msgs::msg::PoseStamped>::iterator closest_pose_detection_begin_;

    bool getRobotPose(std::string path_frame_id, geometry_msgs::msg::PoseStamped& pose);

    static double poseDistance(const geometry_msgs::msg::PoseStamped& pose1,
                               const geometry_msgs::msg::PoseStamped& pose2, double angular_distance_weight);

    static void resamplePath(nav_msgs::msg::Path& path, double spacing_m);
    static void capPath(nav_msgs::msg::Path& path, int max_poses);
};

} // namespace nav3d_behavior_tree