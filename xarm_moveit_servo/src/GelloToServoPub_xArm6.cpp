// src/abs_to_servo.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <unordered_map>
#include <algorithm>
#include <cmath>

using rclcpp::Node;
using control_msgs::msg::JointJog;
using sensor_msgs::msg::JointState;

namespace xarm_moveit_servo
{
class GelloToServoPub : public rclcpp::Node
{
public:
  GelloToServoPub(const rclcpp::NodeOptions& options)
    : Node("gello_to_servo_publisher", options),
      joint_names_({"joint1","joint2","joint3","joint4","joint5","joint6"})
  {
    // --- parameters ---
    declare_parameter<double>("rate_hz", 100.0);
    declare_parameter<double>("deadband", 1e-4);            // rad
    declare_parameter<double>("kp", 4.0);                   // rad/s per rad error
    declare_parameter<double>("max_vel_per_joint", 1.0);    // rad/s cap per joint
    declare_parameter<double>("target_timeout_s", 0.25);    // watchdog seconds

    rate_hz_            = get_parameter("rate_hz").as_double();
    deadband_           = get_parameter("deadband").as_double();
    kp_                 = get_parameter("kp").as_double();
    max_vel_per_joint_  = get_parameter("max_vel_per_joint").as_double();
    target_timeout_s_   = get_parameter("target_timeout_s").as_double();

    // state
    const size_t n = joint_names_.size();
    target_.assign(n, 0.0);
    current_.assign(n, 0.0);

    // --- I/O ---
    js_sub_ = create_subscription<JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](JointState::SharedPtr msg){ on_js(std::move(msg)); });

    // NOTE: keeping your original target topic
    tgt_sub_ = create_subscription<JointState>(
      "gello/joint_states", 10,
      [this](JointState::SharedPtr msg){ on_target(std::move(msg)); });

    // Publish to your Servo JointJog input topic
    pub_ = create_publisher<JointJog>("servo_server/delta_joint_cmds", 10);

    // control loop timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_)),
      [this](){ tick(); });

    // Start Servo (as you had)
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_server/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    RCLCPP_INFO(get_logger(),
      "GelloToServoPub: velocity mode (kp=%.3f, max_vel=%.3f rad/s, deadband=%.1e, rate=%.1f Hz)",
      kp_, max_vel_per_joint_, deadband_, rate_hz_);
  }

private:
  static inline double wrap(double d) { return std::atan2(std::sin(d), std::cos(d)); }
  static inline double clamp(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
  }

  void on_js(const JointState::SharedPtr& msg) {
    // Map incoming joint_states into our fixed order
    std::unordered_map<std::string,double> pos;
    pos.reserve(msg->name.size());
    for (size_t i=0;i<msg->name.size() && i<msg->position.size();++i)
      pos.emplace(msg->name[i], msg->position[i]);

    bool ok = true;
    for (size_t i=0;i<joint_names_.size();++i) {
      auto it = pos.find(joint_names_[i]);
      if (it == pos.end()) { ok = false; break; }
      current_[i] = it->second;
    }
    have_state_ = ok;
  }

  void on_target(const JointState::SharedPtr& msg) {
    // Accept either exact name order or bare positions with correct length
    if (msg->position.size() == joint_names_.size()) {
      if (msg->name.size() == joint_names_.size()) {
        // If names supplied, reorder into our order
        std::unordered_map<std::string,double> pos;
        for (size_t i=0;i<msg->name.size();++i) pos[msg->name[i]] = msg->position[i];
        for (size_t i=0;i<joint_names_.size();++i) target_[i] = pos[joint_names_[i]];
      } else {
        // Assume already in joint1..joint6 order
        for (size_t i=0;i<joint_names_.size();++i) target_[i] = msg->position[i];
      }
      have_target_ = true;
      last_target_time_ = now();
    }
  }

  void tick() {
    if (!have_state_ || !have_target_) return;

    // Watchdog: ignore stale targets (lets Servo stop smoothly)
    if ((now() - last_target_time_).seconds() > target_timeout_s_) return;

    // 1) error = shortest-angle(target - current)
    std::vector<double> err(target_.size());
    bool all_small = true;
    for (size_t i=0;i<err.size();++i) {
      double d = wrap(target_[i] - current_[i]);
      if (std::abs(d) < deadband_) d = 0.0;
      else all_small = false;
      err[i] = d;
    }
    if (all_small) return;

    // 2) P-control -> per-joint velocities, clamped
    std::vector<double> vel(err.size());
    for (size_t i=0;i<vel.size();++i) {
      vel[i] = clamp(kp_ * err[i], -max_vel_per_joint_, +max_vel_per_joint_);
    }

    // 3) Publish JointJog with VELOCITIES ONLY
    JointJog msg;
    msg.header.stamp = now();
    msg.joint_names  = joint_names_;
    msg.velocities   = vel;      // velocity mode
    msg.displacements.clear();   // ensure EMPTY in velocity mode
    pub_->publish(msg);
  }

  // Params/state
  std::vector<std::string> joint_names_;
  std::vector<double> target_, current_;
  bool have_state_{false}, have_target_{false};
  rclcpp::Time last_target_time_;
  double rate_hz_{100.0}, deadband_{1e-4};
  double kp_{4.0}, max_vel_per_joint_{1.0}, target_timeout_s_{0.25};

  // ROS
  rclcpp::Subscription<JointState>::SharedPtr js_sub_, tgt_sub_;
  rclcpp::Publisher<JointJog>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
};
}  // namespace xarm_moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(xarm_moveit_servo::GelloToServoPub)
