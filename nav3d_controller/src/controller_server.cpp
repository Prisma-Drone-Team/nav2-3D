#include "nav3d_controller/controller_server.hpp"

#include "nav3d_core/controller_exceptions.hpp"
#include "nav3d_util/node_utils.hpp"
#include "rcl/time.h"
#include "tf2_ros/create_timer_ros.h"

#include <algorithm>
#include <builtin_interfaces/msg/duration.hpp>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
using nav3d_util::declare_parameter_if_not_declared;

namespace {

rclcpp::NodeOptions withAutoDeclaredParams(const rclcpp::NodeOptions& in) {
    auto out = in;
    out.automatically_declare_parameters_from_overrides(true);
    return out;
}

builtin_interfaces::msg::Duration toMsgDuration(const rclcpp::Duration& d) {
    builtin_interfaces::msg::Duration out;
    out.sec = static_cast<int32_t>(d.seconds());
    const auto ns = d.nanoseconds() - static_cast<int64_t>(out.sec) * 1000000000LL;
    out.nanosec = static_cast<uint32_t>(std::max<int64_t>(0, ns));
    return out;
}

} // namespace

namespace nav3d_controller {

ControllerServer::ControllerServer(const rclcpp::NodeOptions& options)
    : nav3d_util::LifecycleNode("controller_server", "", withAutoDeclaredParams(options)) {
    declare_parameter_if_not_declared(this, "controller_frequency", rclcpp::ParameterValue(50.0));
    declare_parameter_if_not_declared(this, "waypoint_topic", rclcpp::ParameterValue(std::string("offboard/waypoint")));

    declare_parameter_if_not_declared(this, "global_frame", rclcpp::ParameterValue(std::string("map")));
    declare_parameter_if_not_declared(this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    declare_parameter_if_not_declared(this, "transform_tolerance", rclcpp::ParameterValue(0.1));

    declare_parameter_if_not_declared(this, "xyz_goal_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(this, "yaw_goal_tolerance", rclcpp::ParameterValue(0.1));

    declare_parameter_if_not_declared(this, "speed_limit_topic", rclcpp::ParameterValue(std::string("speed_limit")));
    declare_parameter_if_not_declared(this, "default_speed_limit", rclcpp::ParameterValue(0.3));

    declare_parameter_if_not_declared(this, "slowdown_distance", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(this, "min_speed", rclcpp::ParameterValue(0.0));

    // Parametri filtro
    declare_parameter_if_not_declared(this, "ref_jerk_max", rclcpp::ParameterValue(0.35));
    declare_parameter_if_not_declared(this, "ref_acc_max", rclcpp::ParameterValue(0.75));
    declare_parameter_if_not_declared(this, "ref_vel_max", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(this, "ref_omega", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(this, "ref_zita", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(this, "ref_yaw_jerk_max", rclcpp::ParameterValue(0.35));
    declare_parameter_if_not_declared(this, "ref_yaw_acc_max", rclcpp::ParameterValue(0.75));
    declare_parameter_if_not_declared(this, "ref_yaw_vel_max", rclcpp::ParameterValue(1.5));
}

ControllerServer::~ControllerServer() = default;

nav3d_util::CallbackReturn ControllerServer::on_configure(const rclcpp_lifecycle::State& state) {
    (void)state;

    controller_frequency_ = get_parameter("controller_frequency").as_double();
    waypoint_topic_ = get_parameter("waypoint_topic").as_string();

    global_frame_ = get_parameter("global_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();

    xyz_goal_tolerance_ = get_parameter("xyz_goal_tolerance").as_double();
    yaw_goal_tolerance_ = get_parameter("yaw_goal_tolerance").as_double();

    default_speed_limit_ = get_parameter("default_speed_limit").as_double();
    speed_limit_ = default_speed_limit_;

    slowdown_distance_ = get_parameter("slowdown_distance").as_double();
    min_speed_ = get_parameter("min_speed").as_double();

    ref_jerk_max_ = get_parameter("ref_jerk_max").as_double();
    ref_acc_max_ = get_parameter("ref_acc_max").as_double();
    ref_vel_max_ = get_parameter("ref_vel_max").as_double();
    ref_omega_ = get_parameter("ref_omega").as_double();
    ref_zita_ = get_parameter("ref_zita").as_double();
    ref_yaw_jerk_max_ = get_parameter("ref_yaw_jerk_max").as_double();
    ref_yaw_acc_max_ = get_parameter("ref_yaw_acc_max").as_double();
    ref_yaw_vel_max_ = get_parameter("ref_yaw_vel_max").as_double();

    std::string speed_limit_topic = get_parameter("speed_limit_topic").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    waypoint_pub_ = create_publisher<nav3d_msgs::msg::TrajectoryPoint>(waypoint_topic_, qos);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    try {
        rcl_action_server_options_t server_options = rcl_action_server_get_default_options();

        action_server_ = std::make_unique<ActionServer>(shared_from_this(), "follow_path",
                                                        std::bind(&ControllerServer::computeControl, this), nullptr,
                                                        std::chrono::milliseconds(500), true, server_options);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error creating follow_path action server: %s", e.what());
        on_cleanup(state);
        return nav3d_util::CallbackReturn::FAILURE;
    }

    speed_limit_sub_ = create_subscription<nav3d_msgs::msg::SpeedLimit>(
        speed_limit_topic, rclcpp::QoS(10),
        std::bind(&ControllerServer::speedLimitCallback, this, std::placeholders::_1));

    current_path_.clear();
    current_waypoint_index_ = 0;
    path_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    filter_initialized_ = false;
    ref_state_.pos.setZero();
    ref_state_.vel.setZero();
    ref_state_.acc.setZero();
    ref_state_.yaw = 0.0;
    ref_state_.yaw_vel = 0.0;
    ref_state_.yaw_acc = 0.0;

    return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn ControllerServer::on_activate(const rclcpp_lifecycle::State&) {
    action_server_->activate();
    createBond();
    return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn ControllerServer::on_deactivate(const rclcpp_lifecycle::State&) {
    action_server_->deactivate();
    destroyBond();
    return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn ControllerServer::on_cleanup(const rclcpp_lifecycle::State&) {
    action_server_.reset();
    waypoint_pub_.reset();
    speed_limit_sub_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    current_path_.clear();
    current_waypoint_index_ = 0;
    path_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    filter_initialized_ = false;

    return nav3d_util::CallbackReturn::SUCCESS;
}

nav3d_util::CallbackReturn ControllerServer::on_shutdown(const rclcpp_lifecycle::State&) {
    return nav3d_util::CallbackReturn::SUCCESS;
}

// ====================================================================
//  COMPUTECONTROL (definizione che mancava)
// ====================================================================

void ControllerServer::computeControl() {
    try {
        auto goal = action_server_->get_current_goal();
        if (!goal) {
            action_server_->terminate_all();
            return;
        }

        generateTimedTrajectory(goal->path);

        last_valid_cmd_time_ = now();
        rclcpp::WallRate loop_rate(controller_frequency_);

        while (rclcpp::ok()) {
            const auto start_time = now();

            if (!action_server_ || !action_server_->is_server_active()) {
                return;
            }

            if (action_server_->is_cancel_requested()) {
                action_server_->terminate_all();
                return;
            }

            if (action_server_->is_preempt_requested()) {
                action_server_->accept_pending_goal();
                goal = action_server_->get_current_goal();
                if (!goal) {
                    action_server_->terminate_all();
                    return;
                }

                current_path_.clear();
                current_waypoint_index_ = 0;
                generateTimedTrajectory(goal->path);
                last_valid_cmd_time_ = now();
            }

            const auto cmd = getCurrentCommand();
            if (cmd.has_value()) {
                waypoint_pub_->publish(*cmd);
            }

            if (isGoalReached()) {
                current_path_.clear();
                current_waypoint_index_ = 0;
                action_server_->succeeded_current();
                return;
            }

            const auto cycle = now() - start_time;
            if (!loop_rate.sleep()) {
                const double hz = cycle.seconds() > 0.0 ? 1.0 / cycle.seconds() : 0.0;
                RCLCPP_WARN(get_logger(), "Control loop missed rate %.4f Hz (current ~ %.4f Hz).",
                            controller_frequency_, hz);
            }
        }

        action_server_->terminate_all();
        return;

    } catch (nav3d_core::ControllerTFError& e) {
        auto result = std::make_shared<Action::Result>();
        result->error_code = Action::Result::TF_ERROR;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (nav3d_core::PatienceExceeded& e) {
        auto result = std::make_shared<Action::Result>();
        result->error_code = Action::Result::PATIENCE_EXCEEDED;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (nav3d_core::InvalidPath& e) {
        auto result = std::make_shared<Action::Result>();
        result->error_code = Action::Result::INVALID_PATH;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (nav3d_core::ControllerTimedOut& e) {
        auto result = std::make_shared<Action::Result>();
        result->error_code = Action::Result::CONTROLLER_TIMED_OUT;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (nav3d_core::ControllerException& e) {
        auto result = std::make_shared<Action::Result>();
        result->error_code = Action::Result::UNKNOWN;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    } catch (std::exception& e) {
        auto result = std::make_shared<Action::Result>();
        result->error_code = Action::Result::UNKNOWN;
        result->error_msg = e.what();
        action_server_->terminate_current(result);
        return;
    }
}

// ====================================================================
//  FILTRO DI RIFERIMENTO
// ====================================================================

void ControllerServer::resetFilter(const geometry_msgs::msg::Pose & start_pose)
{
    ref_state_.pos = Eigen::Vector3d(
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z
    );
    ref_state_.vel.setZero();
    ref_state_.acc.setZero();

    ref_state_.yaw = tf2::getYaw(start_pose.orientation);
    ref_state_.yaw_vel = 0.0;
    ref_state_.yaw_acc = 0.0;

    filter_initialized_ = true;
}

void ControllerServer::updateFilter(const geometry_msgs::msg::Pose & target_pose,
                                     const geometry_msgs::msg::Twist & target_vel,
                                     double dt)
{
    if (dt <= 0.0 || !filter_initialized_) return;

    // Estrai posizione e yaw target
    Eigen::Vector3d target_pos(target_pose.position.x,
                               target_pose.position.y,
                               target_pose.position.z);
    double target_yaw = tf2::getYaw(target_pose.orientation);

    // ---- Normalizza angoli ----
    target_yaw = angles::normalize_angle(target_yaw);
    ref_state_.yaw = angles::normalize_angle(ref_state_.yaw);

    // ---- Errore minimo di yaw ----
    double yaw_error = angles::shortest_angular_distance(ref_state_.yaw, target_yaw);

    // ---- POSIZIONE: PD con feedforward di velocità ----
    Eigen::Vector3d target_vel_vec(target_vel.linear.x,
                                   target_vel.linear.y,
                                   target_vel.linear.z);

    Eigen::Vector3d pos_error = target_pos - ref_state_.pos;
    Eigen::Vector3d vel_error = ref_state_.vel - target_vel_vec;
    Eigen::Vector3d desired_acc = ref_omega_ * ref_omega_ * pos_error -
                                  2.0 * ref_zita_ * ref_omega_ * vel_error;

    // Limita jerk
    Eigen::Vector3d jerk = (desired_acc - ref_state_.acc) / dt;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(jerk(i)) > ref_jerk_max_) {
            jerk(i) = std::copysign(ref_jerk_max_, jerk(i));
        }
    }
    desired_acc = ref_state_.acc + jerk * dt;

    // Limita accelerazione
    for (int i = 0; i < 3; ++i) {
        if (std::abs(desired_acc(i)) > ref_acc_max_) {
            desired_acc(i) = std::copysign(ref_acc_max_, desired_acc(i));
        }
    }
    ref_state_.acc = desired_acc;

    // Integra velocità
    Eigen::Vector3d new_vel = ref_state_.vel + ref_state_.acc * dt;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(new_vel(i)) > ref_vel_max_) {
            new_vel(i) = std::copysign(ref_vel_max_, new_vel(i));
        }
    }
    ref_state_.vel = new_vel;

    // Integra posizione
    ref_state_.pos += ref_state_.vel * dt;

    // ---- YAW: PD con feedforward, attrito e limitazione adattiva ----
    double target_yaw_vel = target_vel.angular.z;
    double yaw_vel_error = ref_state_.yaw_vel - target_yaw_vel;

    // Attrito extra per smorzare oscillazioni
    const double damping = 0.1;

    double desired_yaw_acc = ref_omega_ * ref_omega_ * yaw_error -
                             2.0 * ref_zita_ * ref_omega_ * yaw_vel_error
                             - damping * ref_state_.yaw_vel;

    // Limita jerk
    double yaw_jerk = (desired_yaw_acc - ref_state_.yaw_acc) / dt;
    if (std::abs(yaw_jerk) > ref_yaw_jerk_max_) {
        yaw_jerk = std::copysign(ref_yaw_jerk_max_, yaw_jerk);
    }
    desired_yaw_acc = ref_state_.yaw_acc + yaw_jerk * dt;

    // Limita accelerazione
    if (std::abs(desired_yaw_acc) > ref_yaw_acc_max_) {
        desired_yaw_acc = std::copysign(ref_yaw_acc_max_, desired_yaw_acc);
    }
    ref_state_.yaw_acc = desired_yaw_acc;

    // ---- Limita velocità angolare in modo adattivo ----
    double yaw_vel_limit = ref_yaw_vel_max_;

    // Se l'errore è piccolo, riduci la velocità massima per evitare overshoot
    if (std::abs(yaw_error) < 0.2) {
        yaw_vel_limit = ref_yaw_vel_max_ * std::max(0.1, std::abs(yaw_error) / 0.2);
    }

    double new_yaw_vel = ref_state_.yaw_vel + ref_state_.yaw_acc * dt;
    if (std::abs(new_yaw_vel) > yaw_vel_limit) {
        new_yaw_vel = std::copysign(yaw_vel_limit, new_yaw_vel);
    }
    ref_state_.yaw_vel = new_yaw_vel;

    ref_state_.yaw += ref_state_.yaw_vel * dt;
    ref_state_.yaw = angles::normalize_angle(ref_state_.yaw);
}

nav3d_msgs::msg::TrajectoryPoint ControllerServer::getFilteredSetpoint()
{
    nav3d_msgs::msg::TrajectoryPoint out;

    geometry_msgs::msg::Pose pose;
    pose.position.x = ref_state_.pos.x();
    pose.position.y = ref_state_.pos.y();
    pose.position.z = ref_state_.pos.z();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, ref_state_.yaw);
    pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = ref_state_.vel.x();
    twist.linear.y = ref_state_.vel.y();
    twist.linear.z = ref_state_.vel.z();
    twist.angular.z = ref_state_.yaw_vel;

    geometry_msgs::msg::Accel accel;
    accel.linear.x = ref_state_.acc.x();
    accel.linear.y = ref_state_.acc.y();
    accel.linear.z = ref_state_.acc.z();

    out.pose = pose;
    out.velocity = twist;
    out.acceleration = accel;
    out.time_from_start = toMsgDuration(this->now() - path_start_time_);

    return out;
}

// ====================================================================
//  GENERAZIONE TRAIETTORIA
// ====================================================================

void ControllerServer::generateTimedTrajectory(const nav_msgs::msg::Path& path) {
    if (path.poses.empty()) {
        throw nav3d_core::InvalidPath("Path is empty.");
    }

    current_path_.clear();
    current_waypoint_index_ = 0;
    current_path_.reserve(path.poses.size());

    auto poses = path.poses;

    const rclcpp::Time start_time = now() + rclcpp::Duration::from_seconds(0.1);
    path_start_time_ = start_time;

    double accumulated_time = 0.0;
    const double v = std::max(1e-3, getSpeedLimit());

    poses[0].header.stamp = start_time;
    current_path_.emplace_back(start_time, poses[0]);

    for (size_t i = 1; i < poses.size(); ++i) {
        const auto& p0 = poses[i - 1].pose.position;
        const auto& p1 = poses[i].pose.position;

        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;
        const double dz = p1.z - p0.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        double dt = dist / v;
        dt = std::max(dt, 0.01);
        accumulated_time += dt;

        const rclcpp::Time t = start_time + rclcpp::Duration::from_seconds(accumulated_time);
        poses[i].header.stamp = t;
        current_path_.emplace_back(t, poses[i]);
    }

    // Inizializza il filtro con la posizione corrente
    geometry_msgs::msg::PoseStamped current_pose;
    if (nav3d_util::getCurrentPose(current_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_)) {
        resetFilter(current_pose.pose);
    } else {
        resetFilter(poses[0].pose);
    }
}

std::optional<nav3d_msgs::msg::TrajectoryPoint> ControllerServer::getCurrentCommand() {
    if (current_path_.size() < 1) {
        return std::nullopt;
    }

    const rclcpp::Time tnow = now();
    const double dt = 1.0 / std::max(1e-3, controller_frequency_);

    if (tnow <= current_path_.front().first) {
        geometry_msgs::msg::PoseStamped current_pose;
        if (nav3d_util::getCurrentPose(current_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_)) {
            resetFilter(current_pose.pose);
        }
        return getFilteredSetpoint();
    }

    if (tnow >= current_path_.back().first) {
        const auto& goal = current_path_.back().second;
        geometry_msgs::msg::Twist zero_twist;
        updateFilter(goal.pose, zero_twist, dt);

        auto out = getFilteredSetpoint();

        const auto& goal_pos = goal.pose.position;
        double dx = ref_state_.pos.x() - goal_pos.x;
        double dy = ref_state_.pos.y() - goal_pos.y;
        double dz = ref_state_.pos.z() - goal_pos.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < xyz_goal_tolerance_) {
            out.velocity.linear.x = 0.0;
            out.velocity.linear.y = 0.0;
            out.velocity.linear.z = 0.0;
            out.velocity.angular.z = 0.0;
        }

        return out;
    }

    while (current_waypoint_index_ + 1 < current_path_.size() &&
           tnow > current_path_[current_waypoint_index_ + 1].first) {
        ++current_waypoint_index_;
    }

    const auto& a = current_path_[current_waypoint_index_];
    const auto& b = current_path_[current_waypoint_index_ + 1];

    const double segment_dt = (b.first - a.first).seconds();
    const double alpha = std::clamp((tnow - a.first).seconds() / std::max(1e-6, segment_dt), 0.0, 1.0);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = a.second.pose.position.x + alpha * (b.second.pose.position.x - a.second.pose.position.x);
    target_pose.position.y = a.second.pose.position.y + alpha * (b.second.pose.position.y - a.second.pose.position.y);
    target_pose.position.z = a.second.pose.position.z + alpha * (b.second.pose.position.z - a.second.pose.position.z);

    tf2::Quaternion q0, q1;
    tf2::fromMsg(a.second.pose.orientation, q0);
    tf2::fromMsg(b.second.pose.orientation, q1);
    tf2::Quaternion q = q0.slerp(q1, alpha);
    q.normalize();
    target_pose.orientation = tf2::toMsg(q);

    geometry_msgs::msg::Twist target_vel;
    target_vel.linear.x = (b.second.pose.position.x - a.second.pose.position.x) / std::max(1e-6, segment_dt);
    target_vel.linear.y = (b.second.pose.position.y - a.second.pose.position.y) / std::max(1e-6, segment_dt);
    target_vel.linear.z = (b.second.pose.position.z - a.second.pose.position.z) / std::max(1e-6, segment_dt);

    const double yaw0 = tf2::getYaw(a.second.pose.orientation);
    const double yaw1 = tf2::getYaw(b.second.pose.orientation);
    const double dyaw = angles::shortest_angular_distance(yaw0, yaw1);
    target_vel.angular.z = dyaw / std::max(1e-6, segment_dt);

    const auto& goal = current_path_.back().second.pose.position;
    const double dxg = goal.x - target_pose.position.x;
    const double dyg = goal.y - target_pose.position.y;
    const double dzg = goal.z - target_pose.position.z;
    const double dist_to_goal = std::sqrt(dxg*dxg + dyg*dyg + dzg*dzg);

    const double dslow = std::max(1e-3, slowdown_distance_);
    const double scale = std::clamp(dist_to_goal / dslow, 0.0, 1.0);

    target_vel.linear.x *= scale;
    target_vel.linear.y *= scale;
    target_vel.linear.z *= scale;
    target_vel.angular.z *= scale;

    if (dist_to_goal <= xyz_goal_tolerance_) {
        target_vel.linear.x = 0.0;
        target_vel.linear.y = 0.0;
        target_vel.linear.z = 0.0;
        target_vel.angular.z = 0.0;
    }

    updateFilter(target_pose, target_vel, dt);

    auto out = getFilteredSetpoint();

    if (dist_to_goal <= xyz_goal_tolerance_) {
        out.velocity.linear.x = 0.0;
        out.velocity.linear.y = 0.0;
        out.velocity.linear.z = 0.0;
        out.velocity.angular.z = 0.0;
    }

    return out;
}

bool ControllerServer::isGoalReached() {
    if (!tf_buffer_) {
        return false;
    }
    if (current_path_.empty()) {
        return false;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav3d_util::getCurrentPose(current_pose, *tf_buffer_, global_frame_, robot_base_frame_,
                                    transform_tolerance_)) {
        return false;
    }

    const auto& goal_pose = current_path_.back().second.pose;
    const auto& cur_pose = current_pose.pose;

    const double dx = cur_pose.position.x - goal_pose.position.x;
    const double dy = cur_pose.position.y - goal_pose.position.y;
    const double dz = cur_pose.position.z - goal_pose.position.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist > xyz_goal_tolerance_) {
        return false;
    }

    const double cur_yaw = tf2::getYaw(cur_pose.orientation);
    const double goal_yaw = tf2::getYaw(goal_pose.orientation);
    const double dyaw = angles::shortest_angular_distance(cur_yaw, goal_yaw);

    return std::fabs(dyaw) <= yaw_goal_tolerance_;
}

void ControllerServer::speedLimitCallback(const nav3d_msgs::msg::SpeedLimit::SharedPtr msg) {
    if (msg->speed_limit > 0.0) {
        speed_limit_ = msg->speed_limit;
    } else {
        speed_limit_ = default_speed_limit_;
    }
}

} // namespace nav3d_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(nav3d_controller::ControllerServer)