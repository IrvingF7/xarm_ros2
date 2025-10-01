// src/gello_to_servo_vel.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <unordered_map>
#include <vector>
#include <string>
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
  explicit GelloToServoPub(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("gello_to_servo_vel", options)
  {
    // --- Parameters (no sign/offset mapping here) ---
    declare_parameter<std::vector<std::string>>("follower_joint_names",
      {"joint1","joint2","joint3","joint4","joint5","joint6"});

    declare_parameter<std::string>("leader_topic", "gello/joint_states");
    declare_parameter<std::string>("follower_states_topic", "/joint_states");
    declare_parameter<std::string>("servo_cmd_topic", "servo_server/delta_joint_cmds");
    declare_parameter<std::string>("servo_start_srv", "/servo_server/start_servo");

    declare_parameter<double>("rate_hz", 200.0);
    declare_parameter<double>("deadband_rad", 1e-4);
    declare_parameter<double>("vel_deadband", 1e-3);
    declare_parameter<double>("kp", 4.0);
    declare_parameter<double>("kd", 0.0);
    declare_parameter<double>("k_ff", 1.0);

    declare_parameter<double>("max_vel_per_joint", 1.0);
    declare_parameter<double>("max_accel_per_joint", 50.0);
    declare_parameter<double>("vel_filter_tau_s", 0.005);

    declare_parameter<double>("leader_timeout_s", 0.25);
    declare_parameter<double>("state_timeout_s", 0.25);
  // Ignore tiny changes in leader joint position between messages (filters jitter before diff)
  declare_parameter<double>("pos_change_threshold_rad", 0.005);

    follower_joint_names_ = get_parameter("follower_joint_names").as_string_array();

    leader_topic_          = get_parameter("leader_topic").as_string();
    follower_states_topic_ = get_parameter("follower_states_topic").as_string();
    servo_cmd_topic_       = get_parameter("servo_cmd_topic").as_string();
    servo_start_srv_       = get_parameter("servo_start_srv").as_string();

    rate_hz_        = get_parameter("rate_hz").as_double();
    deadband_       = get_parameter("deadband_rad").as_double();
    vel_deadband_   = get_parameter("vel_deadband").as_double();
    kp_             = get_parameter("kp").as_double();
    kd_             = get_parameter("kd").as_double();
    k_ff_           = get_parameter("k_ff").as_double();
    vmax_           = get_parameter("max_vel_per_joint").as_double();
    amax_           = get_parameter("max_accel_per_joint").as_double();
    tau_            = get_parameter("vel_filter_tau_s").as_double();
    leader_timeout_ = get_parameter("leader_timeout_s").as_double();
    state_timeout_  = get_parameter("state_timeout_s").as_double();
  pos_change_thresh_rad_ = get_parameter("pos_change_threshold_rad").as_double();

    const size_t N = follower_joint_names_.size();
    follower_pos_.assign(N, 0.0);
    follower_vel_.assign(N, 0.0);
    leader_pos_now_.assign(N, 0.0);
    leader_pos_last_.assign(N, 0.0);
    leader_vel_now_.assign(N, 0.0);
    tmp_err_.assign(N, 0.0);
    cmd_vel_raw_.assign(N, 0.0);
    prev_cmd_vel_.assign(N, 0.0);
    filt_cmd_vel_.assign(N, 0.0);

    // --- I/O ---
    leader_sub_ = create_subscription<JointState>(
      leader_topic_, rclcpp::SensorDataQoS(),
      [this](JointState::SharedPtr msg){ on_leader(std::move(msg)); });

    follower_sub_ = create_subscription<JointState>(
      follower_states_topic_, rclcpp::SensorDataQoS(),
      [this](JointState::SharedPtr msg){ on_follower(std::move(msg)); });
    
    rclcpp::QoS qos_servo_pub(1);
    qos_servo_pub.best_effort();     // Drop if late
    qos_servo_pub.durability_volatile();
    pub_ = create_publisher<JointJog>(servo_cmd_topic_, qos_servo_pub);

    // Control loop
    const double hz = std::max(20.0, rate_hz_);
    loop_dt_ = 1.0 / hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(loop_dt_),
      [this](){ tick(); });

    // Optionally start Servo
    servo_start_ = create_client<std_srvs::srv::Trigger>(servo_start_srv_);
    (void)servo_start_->wait_for_service(std::chrono::seconds(1));
    if (servo_start_->service_is_ready())
      servo_start_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    RCLCPP_INFO(get_logger(),
      "Gello→Servo velocity mode: N=%zu, rate=%.1f Hz, kp=%.3f, kd=%.3f, k_ff=%.3f, vmax=%.2f, amax=%.2f, tau=%.3f, dpos_thr=%.6f",
      N, hz, kp_, kd_, k_ff_, vmax_, amax_, tau_, pos_change_thresh_rad_);
  }

private:
  // Helpers
  static inline double wrap(double d){ return std::atan2(std::sin(d), std::cos(d)); }
  static inline double clamp(double x,double lo,double hi){ return std::max(lo,std::min(hi,x)); }

  // Reorder into follower order. If leader msg has names, use them.
  // If no names but positions length matches, assume same order.
  bool reorder_into_follower(const JointState& js, std::vector<double>& out_pos) const
  {
    const size_t N = follower_joint_names_.size();
    out_pos.resize(N);

    if (!js.name.empty()) {
      std::unordered_map<std::string, size_t> idx;
      idx.reserve(js.name.size());
      for (size_t i=0;i<js.name.size();++i) idx[js.name[i]] = i;
      for (size_t i=0;i<N;++i) {
        auto it = idx.find(follower_joint_names_[i]);
        if (it == idx.end() || it->second >= js.position.size()) return false;
        out_pos[i] = js.position[it->second];
      }
      return true;
    } else if (js.position.size() == N) {
      std::copy(js.position.begin(), js.position.end(), out_pos.begin());
      return true;
    }
    return false;
  }

  void on_follower(const JointState::SharedPtr& msg)
  {
    last_follower_stamp_ = now();

    // Positions (reordered) and velocities (if provided, also reorder by name; else zeros)
    if (!reorder_into_follower(*msg, follower_pos_)) {
      have_follower_ = false;
      return;
    }

    // Velocities
    follower_vel_.assign(follower_vel_.size(), 0.0);
    if (!msg->name.empty() && msg->velocity.size() == msg->name.size()) {
      std::unordered_map<std::string, size_t> idx;
      idx.reserve(msg->name.size());
      for (size_t i=0;i<msg->name.size();++i) idx[msg->name[i]] = i;
      for (size_t i=0;i<follower_joint_names_.size();++i) {
        auto it = idx.find(follower_joint_names_[i]);
        if (it != idx.end()) follower_vel_[i] = msg->velocity[it->second];
      }
    } else if (msg->name.empty() && msg->velocity.size() == follower_vel_.size()) {
      std::copy(msg->velocity.begin(), msg->velocity.end(), follower_vel_.begin());
    }

    have_follower_ = true;
  }

void on_leader(const JointState::SharedPtr& msg)
{
  // Expect Gello already aligned; reorder into follower order
  if (!reorder_into_follower(*msg, leader_pos_now_)) {
    have_leader_ = false;
    return;
  }

  // Get current ROS time from the node clock
  const rclcpp::Time t_now = now();

  double dt = 0.0;
  if (have_leader_last_) {
    // Only subtract when both stamps come from the same clock
    dt = (t_now - last_leader_stamp_).seconds();
  }
  last_leader_stamp_ = t_now;

  // Ignore tiny position steps on leader to reduce jitter before differentiating
  if (have_leader_last_ && pos_change_thresh_rad_ > 0.0) {
    for (size_t i=0;i<leader_pos_now_.size();++i) {
      const double d = wrap(leader_pos_now_[i] - leader_pos_last_[i]);
      if (std::fabs(d) < pos_change_thresh_rad_) {
        leader_pos_now_[i] = leader_pos_last_[i];
      }
    }
  }

  // Finite-difference leader velocity with unwrap
  if (have_leader_last_ && dt > 1e-4 && dt < 1.0) {
    for (size_t i=0;i<leader_pos_now_.size();++i) {
      const double d = wrap(leader_pos_now_[i] - leader_pos_last_[i]);
      leader_vel_now_[i] = d / dt;
    }
  } else {
    std::fill(leader_vel_now_.begin(), leader_vel_now_.end(), 0.0);
  }

  leader_pos_last_ = leader_pos_now_;
  have_leader_ = true;
  have_leader_last_ = true;
}

  void tick()
  {
    if (!have_follower_ || !have_leader_) return;

    // Watchdogs
    if ((now() - last_leader_stamp_).seconds()   > leader_timeout_) return;
    if ((now() - last_follower_stamp_).seconds() > state_timeout_)  return;

    const size_t N = follower_joint_names_.size();

    // 1) error
    bool all_small = true;
    for (size_t i=0;i<N;++i) {
      double e = wrap(leader_pos_now_[i] - follower_pos_[i]);
      if (std::fabs(e) < deadband_) e = 0.0; else all_small = false;
      tmp_err_[i] = e;
    }

    // 2) velocity command = FF + P - D
    for (size_t i=0;i<N;++i) {
      const double vff = (std::fabs(leader_vel_now_[i]) < vel_deadband_) ? 0.0 : (k_ff_ * leader_vel_now_[i]);
      const double vp  = kp_ * tmp_err_[i];
      const double vd  = -kd_ * follower_vel_[i];
      cmd_vel_raw_[i]  = vff + vp + vd;
    }

    // 3) hard vel cap
    for (double& v : cmd_vel_raw_) v = clamp(v, -vmax_, +vmax_);

    // 4) slew-rate (accel) limit
    for (size_t i=0;i<N;++i) {
      const double dv = cmd_vel_raw_[i] - prev_cmd_vel_[i];
      const double dv_max = amax_ * loop_dt_;
      prev_cmd_vel_[i] += clamp(dv, -dv_max, +dv_max);
    }

    // 5) EMA smoothing
    const double alpha = (tau_ <= 1e-6) ? 1.0 : (loop_dt_ / (tau_ + loop_dt_));
    for (size_t i=0;i<N;++i) {
      filt_cmd_vel_[i] += alpha * (prev_cmd_vel_[i] - filt_cmd_vel_[i]);
    }

    // 6) skip spam if tiny
    bool all_zeroish = true;
    for (double v : filt_cmd_vel_) if (std::fabs(v) > 1e-6) { all_zeroish = false; break; }
    if (all_small && all_zeroish) return;

    // 7) publish velocities only
    JointJog jj;
    jj.header.stamp = now();
    jj.joint_names  = follower_joint_names_;
    jj.velocities   = filt_cmd_vel_;
    jj.displacements.clear();   // IMPORTANT: empty in velocity mode
    pub_->publish(jj);
  }

  // Params
  std::vector<std::string> follower_joint_names_;
  std::string leader_topic_, follower_states_topic_, servo_cmd_topic_, servo_start_srv_;
  double rate_hz_{200.0}, loop_dt_{0.005};
  double deadband_{1e-4}, vel_deadband_{1e-3};
  double kp_{4.0}, kd_{0.0}, k_ff_{1.0};
  double vmax_{1.0}, amax_{10.0}, tau_{0.02};
  double leader_timeout_{0.25}, state_timeout_{0.25};
  double pos_change_thresh_rad_{0.0};

  // State
  std::vector<double> follower_pos_, follower_vel_;
  std::vector<double> leader_pos_now_, leader_pos_last_, leader_vel_now_;
  std::vector<double> tmp_err_, cmd_vel_raw_, prev_cmd_vel_, filt_cmd_vel_;
  rclcpp::Time last_leader_stamp_, last_leader_stamp_prev_, last_follower_stamp_;
  bool have_follower_{false}, have_leader_{false}, have_leader_last_{false};

  // ROS
  rclcpp::Subscription<JointState>::SharedPtr leader_sub_, follower_sub_;
  rclcpp::Publisher<JointJog>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_;
};
} // namespace xarm_moveit_servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(xarm_moveit_servo::GelloToServoPub)
