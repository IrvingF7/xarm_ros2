// src/multi_ee_pose_twist_component.cpp
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

    get_parameter("planning_group", group_);
    get_parameter("tcp_link", tcp_link_);
    get_parameter("eef_link", eef_link_);
    get_parameter("base_frame", base_frame_);
    get_parameter("twist_frame", twist_frame_);

    // Load model from existing robot_description
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    model_ = loader.getModel();
    if (!model_) throw std::runtime_error("Failed to load RobotModel from 'robot_description'.");
    state_ = std::make_shared<moveit::core::RobotState>(model_);
    jmg_   = model_->getJointModelGroup(group_);
    if (!jmg_) throw std::runtime_error("Bad planning_group: " + group_);

    model_frame_ = model_->getModelFrame();
    if (base_frame_.empty()) base_frame_ = model_frame_;

    // Resolve link models
    add_link_if_ok("tcp", tcp_link_);
    add_link_if_ok("eef", eef_link_);
    if (link_models_.empty()) throw std::runtime_error("Neither tcp_link nor eef_link found in model.");

    // Publishers per target
    for (const auto& kv : link_models_) {
      const auto& key = kv.first; // "tcp" or "eef"
      pose_pub_[key]  = create_publisher<geometry_msgs::msg::PoseStamped>(key + std::string("/pose"), 10);
      twist_pub_[key] = create_publisher<geometry_msgs::msg::TwistStamped>(key + std::string("/twist"), 10);
    }

    // Cache group var names ordering
    group_vars_ = jmg_->getVariableNames();

    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      std::bind(&EEPublisher::on_js, this, std::placeholders::_1));
  }

private:
  void add_link_if_ok(const std::string& key, const std::string& link_name) {
    if (link_name.empty()) return;
    const auto* lm = model_->getLinkModel(link_name);
    if (!lm) {
      RCLCPP_WARN(get_logger(), "Link '%s' not found; skipping '%s'.", link_name.c_str(), key.c_str());
      return;
    }
    link_models_[key] = lm;
    link_names_[key]  = link_name;
  }

  void on_js(const sensor_msgs::msg::JointState::SharedPtr js) {
    if (js->name.size() != js->position.size()) return;

    // Update state & transforms
    state_->setVariablePositions(js->name, js->position);
    state_->update();

    // Build dq (from driver or FD fallback)
    Eigen::VectorXd dq(group_vars_.size()); dq.setZero();
    bool have_vel = (js->velocity.size() == js->name.size());
    name_to_pos_.clear(); for (size_t i=0;i<js->name.size();++i) name_to_pos_[js->name[i]] = js->position[i];
    if (have_vel) { name_to_vel_.clear(); for (size_t i=0;i<js->name.size();++i) name_to_vel_[js->name[i]] = js->velocity[i]; }
    double dt = 0.0;
    if (!have_vel) {
      if (!prev_stamp_.nanoseconds()) { prev_pos_ = name_to_pos_; prev_stamp_ = js->header.stamp; return; }
      dt = (rclcpp::Time(js->header.stamp) - prev_stamp_).seconds(); if (dt <= 0.0) return;
    }
    for (size_t i=0;i<group_vars_.size();++i){
      const auto& v = group_vars_[i];
      auto itp = name_to_pos_.find(v); if (itp == name_to_pos_.end()) continue;
      dq[i] = have_vel ? name_to_vel_[v] : (itp->second - prev_pos_[v]) / dt;
    }

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
      pose_pub_[key]->publish(ps);

      // Twist via J(q)*dq (spatial, model frame) → convert to desired frame
      Eigen::MatrixXd J; state_->getJacobian(jmg_, link, Eigen::Vector3d::Zero(), J); // 6xN, spatial in model frame
      Eigen::VectorXd V = J * dq;                            // in model frame
      if (base_frame_ != model_frame_) V = adjoint(T_B_W) * V; // to base (spatial)

      if (twist_frame_ == "body") {
        // Convert spatial (at L, expressed in base) → body (expressed in L)
        Eigen::Isometry3d T_L_B = T_B_L.inverse();
        V = adjoint(T_L_B) * V;
      }

      geometry_msgs::msg::TwistStamped tw;
      tw.header = ps.header;
      tw.twist.linear.x = V(0); tw.twist.linear.y = V(1); tw.twist.linear.z = V(2);
      tw.twist.angular.x = V(3); tw.twist.angular.y = V(4); tw.twist.angular.z = V(5);
      twist_pub_[key]->publish(tw);
    }

    if (!have_vel) { prev_pos_ = name_to_pos_; prev_stamp_ = js->header.stamp; }
  }

  // Params / model
  std::string group_, tcp_link_, eef_link_, base_frame_, twist_frame_, model_frame_;
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
  rclcpp::Time prev_stamp_{0,0,get_clock()->get_clock_type()};
};

} // namespace xarm_moveit_servo

RCLCPP_COMPONENTS_REGISTER_NODE(xarm_moveit_servo::EEPublisher)
