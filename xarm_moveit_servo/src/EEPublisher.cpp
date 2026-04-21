// publish end-effector pose and twist based on joint_states. In meter and rad
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <set>

namespace {
Eigen::Matrix<double,6,6> adjoint(const Eigen::Isometry3d& T){
  const auto& R = T.rotation(); const auto& p = T.translation();
  Eigen::Matrix3d px; px << 0, -p.z(),  p.y(),  p.z(), 0, -p.x(), -p.y(), p.x(), 0;
  Eigen::Matrix<double,6,6> Ad; Ad.setZero();
  Ad.block<3,3>(0,0)=R; Ad.block<3,3>(3,0)=px*R; Ad.block<3,3>(3,3)=R;
  return Ad;
}
}

namespace xarm_moveit_servo
{
class EEPublisher : public rclcpp::Node {
public:
  explicit EEPublisher(const rclcpp::NodeOptions& opts) : Node("ee_publisher", opts) {
    declare_parameter<std::string>("planning_group", "manipulator");
    declare_parameter<std::string>("tcp_link", "link_tcp");
    declare_parameter<std::string>("eef_link", "link_eef");
    declare_parameter<std::string>("base_frame", "");
    declare_parameter<std::string>("twist_frame", "spatial"); // "spatial" or "body"
    declare_parameter<std::string>("use_fake_hardware", "false");
  
    get_parameter("planning_group", group_);
    get_parameter("tcp_link", tcp_link_);
    get_parameter("eef_link", eef_link_);
    get_parameter("base_frame", base_frame_);
    get_parameter("twist_frame", twist_frame_);
    get_parameter("use_fake_hardware", use_fake_hardware_);

    // Publishers per target (link_models_ not populated yet; real creation happens in add_link_if_ok)
    for (const auto& kv : link_models_) {
      const auto& key = kv.first; // "tcp" or "eef"
      pose_pub_[key]  = create_publisher<geometry_msgs::msg::PoseStamped>(key + std::string("/pose"), 10);
      twist_pub_[key] = create_publisher<geometry_msgs::msg::TwistStamped>(key + std::string("/twist"), 10);
    }
    
    // Create a dedicated callback group so a MT executor can run us concurrently
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    // TODO: see if simply using joint_states is sufficient for both real and fake hardware
    if (use_fake_hardware_ == "true") {
      RCLCPP_INFO(get_logger(), "Using fake hardware; EE calculated based on /joint_states.");
      joint_state_topic_ = "joint_states";
    } else {
      RCLCPP_INFO(get_logger(), "Using real hardware; EE calculated based on /xarm/joint_states.");
      joint_state_topic_ = "joint_states";
    }
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&EEPublisher::on_js, this, std::placeholders::_1), sub_opts);

    // IMPORTANT: don’t call shared_from_this() here. Defer init:
    init_timer_ = create_wall_timer(std::chrono::milliseconds(0),
      std::bind(&EEPublisher::late_init, this));
  }

private:
  void late_init() {
    if (initialized_) return;
    // Safe now: the component is owned by a shared_ptr
    robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
    model_ = robot_model_loader_->getModel();
    if (!model_) { RCLCPP_ERROR(get_logger(), "Failed to load robot model"); return; }

    model_frame_ = model_->getModelFrame();
    jmg_ = model_->getJointModelGroup(group_);
    if (!jmg_) { RCLCPP_ERROR(get_logger(), "Bad planning_group: %s", group_.c_str()); return; }

    // Create a robot state
    state_ = std::make_shared<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();
    state_->update();

    // Resolve links (tcp/eef), cache group vars, etc…
    // Resolve link models
    add_link_if_ok("tcp", tcp_link_);
    add_link_if_ok("eef", eef_link_);
    if (link_models_.empty()) throw std::runtime_error("Neither tcp_link nor eef_link found in model.");

    // Cache group var names ordering
    group_vars_ = jmg_->getVariableNames();
    dof_ = jmg_->getVariableCount();
    // Pre-allocations
    dq_.resize(dof_); dq_.setZero();
    q_prev_.resize(dof_); q_prev_.setZero();
    J_.resize(6, dof_);   // preallocate once
    js_index_.resize(dof_, -1);

    initialized_ = true;
    init_timer_->cancel();  // one-shot
  }
  void add_link_if_ok(const std::string& key, const std::string& link_name) {
    if (link_name.empty()) return;
    const auto* lm = model_->getLinkModel(link_name);
    if (!lm) {
      RCLCPP_WARN(get_logger(), "Link '%s' not found; skipping '%s'.", link_name.c_str(), key.c_str());
      return;
    }
    link_models_[key] = lm;
    link_names_[key]  = link_name;

    // Create publishers for this key if not already created
    if (!pose_pub_.count(key) || !pose_pub_[key]) {
      pose_pub_[key] = create_publisher<geometry_msgs::msg::PoseStamped>(std::string("/xarm/") + key + std::string("_pose"), 10);
    }
    if (!twist_pub_.count(key) || !twist_pub_[key]) {
      twist_pub_[key] = create_publisher<geometry_msgs::msg::TwistStamped>(std::string("/xarm/") + key + std::string("_twist"), 10);
    }
  }

  void on_js(const sensor_msgs::msg::JointState::SharedPtr js) {
    if (!initialized_ || !state_) return;
    if (js->name.size() != js->position.size()) return;

    // Build name->index once to avoid per-call unordered_map overhead
    if (!idx_ready_) {
      std::unordered_map<std::string,int> name2i; name2i.reserve(js->name.size());
      for (int i=0; i<(int)js->name.size(); ++i) name2i[js->name[i]] = i;
      for (size_t k=0; k<group_vars_.size(); ++k) {
        auto it = name2i.find(group_vars_[k]);
        if (it == name2i.end()) {
          RCLCPP_WARN(get_logger(), "Joint %s not in joint_states; skipping frame.", group_vars_[k].c_str());
          return;
        }
        js_index_[k] = it->second;
      }
      idx_ready_ = true;
    }

    // Extract q in group order
    tmp_q_.resize(dof_);
    for (size_t k=0; k<group_vars_.size(); ++k)
      tmp_q_[k] = js->position[js_index_[k]];

    state_->setJointGroupPositions(jmg_, tmp_q_);
    state_->update();  // (you may try updateLinkTransforms() only)

    // dq
    dq_.setZero();
    bool have_vel = (js->velocity.size() == js->name.size());
    double dt = 0.0;
    if (!have_vel) {
      if (!prev_stamp_.nanoseconds()) { q_prev_ = Eigen::Map<const Eigen::VectorXd>(tmp_q_.data(), dof_); prev_stamp_ = js->header.stamp; return; }
      dt = (rclcpp::Time(js->header.stamp) - prev_stamp_).seconds();
      if (dt <= 0.0) return;
      Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(tmp_q_.data(), dof_);
      dq_ = (q - q_prev_) / dt;
    } else {
      for (size_t k=0; k<group_vars_.size(); ++k)
        dq_[k] = js->velocity[js_index_[k]];
    }

    // Optional short-circuit if nothing is listening
    size_t subs = 0;
    for (auto& kv : pose_pub_)  if (kv.second) subs += kv.second->get_subscription_count();
    for (auto& kv : twist_pub_) if (kv.second) subs += kv.second->get_subscription_count();
    if (subs == 0) { if (!have_vel){ q_prev_ = Eigen::Map<const Eigen::VectorXd>(tmp_q_.data(), dof_); prev_stamp_ = js->header.stamp; } return; }


    // Base transform (model → base)
    Eigen::Isometry3d T_W_B = Eigen::Isometry3d::Identity();
    if (base_frame_ != model_frame_) {
      const auto* base_link = model_->getLinkModel(base_frame_);
      if (!base_link) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "base_frame '%s' not found; using model frame '%s'.", base_frame_.c_str(), model_frame_.c_str());
        base_frame_ = model_frame_;
      } else {
        T_W_B = state_->getGlobalLinkTransform(base_link);
      }
    }
    Eigen::Isometry3d T_B_W = T_W_B.inverse();

    // For each target link (tcp/eef)
    for (const auto& kv : link_models_) {
      const std::string& key = kv.first;
      const moveit::core::LinkModel* link = kv.second;

      // Pose in base frame
      Eigen::Isometry3d T_W_L = state_->getGlobalLinkTransform(link);
      Eigen::Isometry3d T_B_L = (base_frame_ == model_frame_) ? T_W_L : (T_B_W * T_W_L);

      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = base_frame_;
      ps.header.stamp = js->header.stamp;
      ps.pose.position.x = T_B_L.translation().x();
      ps.pose.position.y = T_B_L.translation().y();
      ps.pose.position.z = T_B_L.translation().z();
      Eigen::Quaterniond q(T_B_L.rotation());
      ps.pose.orientation.w = q.w(); ps.pose.orientation.x = q.x();
      ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z();
      auto pp = pose_pub_.find(key);
      if (pp != pose_pub_.end() && pp->second) pp->second->publish(ps);

      // Jacobian: reuse pre-allocated J_
      state_->getJacobian(jmg_, link, Eigen::Vector3d::Zero(), J_ /*, use_quaternion=false*/);
      Eigen::VectorXd V = J_ * dq_;
      if (base_frame_ != model_frame_) V = adjoint(T_B_W) * V;
      if (twist_frame_ == "body") {
        Eigen::Isometry3d T_L_B = T_B_L.inverse();
        V = adjoint(T_L_B) * V;
      }

      geometry_msgs::msg::TwistStamped tw;
      tw.header = ps.header;
      tw.twist.linear.x = V(0); tw.twist.linear.y = V(1); tw.twist.linear.z = V(2);
      tw.twist.angular.x = V(3); tw.twist.angular.y = V(4); tw.twist.angular.z = V(5);
      auto tp = twist_pub_.find(key);
      if (tp != twist_pub_.end() && tp->second) tp->second->publish(tw);
    }

    if (!have_vel) { q_prev_ = Eigen::Map<const Eigen::VectorXd>(tmp_q_.data(), dof_); prev_stamp_ = js->header.stamp; }
  }

  // Params / model
  bool initialized_{false};
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  size_t dof_{0};
  std::vector<int> js_index_;
  bool idx_ready_{false};
  std::vector<double> tmp_q_;
  Eigen::VectorXd dq_, q_prev_;
  Eigen::MatrixXd J_;

  std::string group_, tcp_link_, eef_link_, base_frame_, twist_frame_, model_frame_, use_fake_hardware_, joint_state_topic_;
  moveit::core::RobotModelPtr model_;
  moveit::core::RobotStatePtr state_;
  const moveit::core::JointModelGroup* jmg_{nullptr};
  std::unordered_map<std::string, const moveit::core::LinkModel*> link_models_; // key -> model
  std::unordered_map<std::string, std::string> link_names_;                     // key -> name
  std::vector<std::string> group_vars_;

  // I/O
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pub_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_pub_;

  // FD helpers
  std::unordered_map<std::string,double> name_to_pos_, name_to_vel_, prev_pos_;
  rclcpp::Time prev_stamp_;
};

} // namespace xarm_moveit_servo

RCLCPP_COMPONENTS_REGISTER_NODE(xarm_moveit_servo::EEPublisher)
