// Copyright 2026 B-Robotized
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "joint_torque_wrench_estimator/wrench_estimator_broadcaster.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <controller_interface/helpers.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>

namespace joint_torque_wrench_estimator
{

namespace
{
constexpr std::array<const char *, 6> kFtsInterfaceSuffixes{"force.x",  "force.y",  "force.z",
                                                            "torque.x", "torque.y", "torque.z"};

// First-order LPF coefficient: alpha = 1 - exp(-dt * 2*pi*fc).
// cutoff_hz <= 0 means "no filter" (return 1.0 -> passthrough on first sample,
// caller handles that case explicitly).
double lpf_alpha(double dt, double cutoff_hz)
{
  if (cutoff_hz <= 0.0 || dt <= 0.0)
  {
    return 1.0;
  }
  return 1.0 - std::exp(-dt * 2.0 * M_PI * cutoff_hz);
}
}  // namespace

WrenchEstimatorBroadcaster::WrenchEstimatorBroadcaster() = default;

controller_interface::CallbackReturn WrenchEstimatorBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<wrench_estimator_broadcaster::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WrenchEstimatorBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  // ── Parse URDF ──────────────────────────────────────────────────────
  const std::string urdf = get_robot_description();
  if (urdf.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "robot_description is empty; cannot build kinematic model.");
    return controller_interface::CallbackReturn::ERROR;
  }

  pinocchio::Model full_model;
  try
  {
    pinocchio::urdf::buildModelFromXML(urdf, full_model);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF with Pinocchio: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!full_model.existFrame(params_.base_link))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "base_link '%s' not found in URDF.", params_.base_link.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!full_model.existFrame(params_.tip_link))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "tip_link '%s' not found in URDF.", params_.tip_link.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // ── Identify chain joints in full model ─────────────────────────────
  const auto base_frame_id_full = full_model.getFrameId(params_.base_link);
  const auto tip_frame_id_full = full_model.getFrameId(params_.tip_link);
  const auto base_parent_joint = full_model.frames[base_frame_id_full].parentJoint;

  std::set<pinocchio::JointIndex> chain_set;
  {
    auto j = full_model.frames[tip_frame_id_full].parentJoint;
    while (j != base_parent_joint)
    {
      if (j == 0)
      {
        RCLCPP_ERROR(
          get_node()->get_logger(), "base_link '%s' is not an ancestor of tip_link '%s'.",
          params_.base_link.c_str(), params_.tip_link.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      chain_set.insert(j);
      j = full_model.parents[j];
    }
  }

  // ── Lock everything not in the chain ────────────────────────────────
  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (pinocchio::JointIndex i = 1;
       i < static_cast<pinocchio::JointIndex>(full_model.joints.size()); ++i)
  {
    if (chain_set.find(i) == chain_set.end())
    {
      joints_to_lock.push_back(i);
    }
  }

  try
  {
    const Eigen::VectorXd q_default = pinocchio::neutral(full_model);
    pinocchio::buildReducedModel(full_model, joints_to_lock, q_default, model_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build reduced model: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Joint names in the reduced model's joint-id order.
  joint_names_.clear();
  joint_names_.reserve(model_.joints.size() - 1);
  for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(model_.joints.size());
       ++i)
  {
    joint_names_.push_back(model_.names[i]);
  }
  n_joints_ = joint_names_.size();

  if (n_joints_ == 0 || static_cast<std::size_t>(model_.nv) != n_joints_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Reduced model has nv=%d, expected %zu moving joints.", model_.nv,
      n_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(), "Reduced chain %s -> %s, %zu joints:", params_.base_link.c_str(),
    params_.tip_link.c_str(), n_joints_);
  for (const auto & name : joint_names_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "  - %s", name.c_str());
  }

  if (!model_.existFrame(params_.base_link) || !model_.existFrame(params_.tip_link))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Reduced model lost base_link or tip_link frame.");
    return controller_interface::CallbackReturn::ERROR;
  }
  base_frame_id_ = model_.getFrameId(params_.base_link);
  tip_frame_id_ = model_.getFrameId(params_.tip_link);

  data_ = std::make_unique<pinocchio::Data>(model_);

  // ── Set gravity (rotate from base_link frame to model root frame) ───
  // At neutral config the world -> base_link transform is fixed (the fixed
  // joints above base_link don't move). Compute it once.
  {
    const Eigen::VectorXd q_neutral = pinocchio::neutral(model_);
    pinocchio::forwardKinematics(model_, *data_, q_neutral);
    pinocchio::updateFramePlacement(model_, *data_, base_frame_id_);
    const Eigen::Matrix3d R_world_base = data_->oMf[base_frame_id_].rotation();
    const Eigen::Vector3d g_in_base(
      params_.gravity_vector[0], params_.gravity_vector[1], params_.gravity_vector[2]);
    model_.gravity.linear() = R_world_base * g_in_base;
    RCLCPP_INFO(
      get_node()->get_logger(), "Gravity in model root frame: [%.3f, %.3f, %.3f]",
      model_.gravity.linear().x(), model_.gravity.linear().y(), model_.gravity.linear().z());
  }

  // ── Resolve observer gain to per-joint vector ───────────────────────
  K_O_.resize(n_joints_);
  if (params_.observer_gain.size() == 1)
  {
    K_O_.setConstant(params_.observer_gain[0]);
  }
  else if (params_.observer_gain.size() == n_joints_)
  {
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      K_O_[i] = params_.observer_gain[i];
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "observer_gain size %zu doesn't match n_joints=%zu (or 1 for broadcast).",
      params_.observer_gain.size(), n_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }

  // ── Resolve torque source: interface names and unit scale ───────────
  torque_interface_names_.clear();
  torque_interface_names_.reserve(n_joints_);
  if (params_.torque_state_interfaces.empty())
  {
    for (const auto & name : joint_names_)
    {
      torque_interface_names_.push_back(name + "/effort");
    }
  }
  else if (params_.torque_state_interfaces.size() == n_joints_)
  {
    torque_interface_names_ = params_.torque_state_interfaces;
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "torque_state_interfaces size %zu doesn't match n_joints=%zu "
      "(or 0 to default to <joint>/effort).",
      params_.torque_state_interfaces.size(), n_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }

  torque_scale_.resize(n_joints_);
  if (params_.torque_scale.size() == 1)
  {
    torque_scale_.setConstant(params_.torque_scale[0]);
  }
  else if (params_.torque_scale.size() == n_joints_)
  {
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      torque_scale_[i] = params_.torque_scale[i];
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "torque_scale size %zu doesn't match n_joints=%zu (or 1 for broadcast).",
      params_.torque_scale.size(), n_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < n_joints_; ++i)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "joint '%s' torque from '%s' (scale %.6g)", joint_names_[i].c_str(),
      torque_interface_names_[i].c_str(), torque_scale_[i]);
  }

  // ── Allocate buffers ────────────────────────────────────────────────
  q_.setZero(n_joints_);
  qd_.setZero(n_joints_);
  qd_filtered_.setZero(n_joints_);
  tau_meas_.setZero(n_joints_);
  tau_filtered_.setZero(n_joints_);
  beta_.setZero(n_joints_);
  p_.setZero(n_joints_);
  p_integral_.setZero(n_joints_);
  r_.setZero(n_joints_);
  tau_ext_.setZero(n_joints_);
  J_.setZero(kCartesianDim, n_joints_);
  I6_.setIdentity();
  JJt_.setZero();
  JJt_alphaI_.setZero();
  wrench_raw_.setZero();
  wrench_filtered_.setZero();
  for (std::size_t i = 0; i < kCartesianDim; ++i)
  {
    wrench_deadband_[static_cast<Eigen::Index>(i)] = params_.wrench_deadband[i];
  }

  // ── Publisher ───────────────────────────────────────────────────────
  wrench_pub_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/wrench", rclcpp::SystemDefaultsQoS());
  rt_wrench_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
      wrench_pub_);

  payload_pub_ = get_node()->create_publisher<geometry_msgs::msg::InertiaStamped>(
    "~/payload", rclcpp::SystemDefaultsQoS());
  rt_payload_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::InertiaStamped>>(
      payload_pub_);

  gravity_magnitude_ =
    Eigen::Vector3d(params_.gravity_vector[0], params_.gravity_vector[1], params_.gravity_vector[2])
      .norm();
  if (gravity_magnitude_ < 1e-6)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "gravity_vector has ~zero magnitude.");
    return controller_interface::CallbackReturn::ERROR;
  }

  exported_wrench_.fill(0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
WrenchEstimatorBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
WrenchEstimatorBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(3 * joint_names_.size());
  for (std::size_t j = 0; j < joint_names_.size(); ++j)
  {
    config.names.push_back(joint_names_[j] + "/position");
    config.names.push_back(joint_names_[j] + "/velocity");
    config.names.push_back(torque_interface_names_[j]);
  }
  return config;
}

std::vector<hardware_interface::CommandInterface>
WrenchEstimatorBroadcaster::on_export_reference_interfaces()
{
  return {};
}

std::vector<hardware_interface::StateInterface>
WrenchEstimatorBroadcaster::on_export_state_interfaces()
{
  // ros2_control mandates that exported state-interface prefixes begin with
  // the controller's own name. The instance name therefore IS the sensor
  // name from downstream's perspective.
  const std::string prefix = get_node()->get_name();
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(kCartesianDim);
  for (std::size_t i = 0; i < kCartesianDim; ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        prefix, std::string(kFtsInterfaceSuffixes[i]), &exported_wrench_[i]));
  }
  return state_interfaces;
}

bool WrenchEstimatorBroadcaster::on_set_chained_mode(bool /*chained_mode*/) { return true; }

controller_interface::CallbackReturn WrenchEstimatorBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Build a name->index map over the loaned state interfaces, then resolve
  // each joint's {position, velocity, effort} once. This is O(n) lookups,
  // done at activation only — update() will index directly.
  joint_si_idx_.assign(n_joints_, {0, 0, 0});

  for (std::size_t j = 0; j < n_joints_; ++j)
  {
    const auto & jname = joint_names_[j];
    bool found_pos = false;
    bool found_vel = false;
    bool found_eff = false;
    for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
    {
      const auto & si_name = state_interfaces_[i].get_name();
      if (si_name == jname + "/position")
      {
        joint_si_idx_[j][0] = i;
        found_pos = true;
      }
      else if (si_name == jname + "/velocity")
      {
        joint_si_idx_[j][1] = i;
        found_vel = true;
      }
      else if (si_name == torque_interface_names_[j])
      {
        joint_si_idx_[j][2] = i;
        found_eff = true;
      }
    }
    if (!found_pos || !found_vel || !found_eff)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Could not resolve {position, velocity, torque} for joint '%s' "
        "(torque interface '%s').",
        jname.c_str(), torque_interface_names_[j].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // Reset all filter/observer state so re-activation starts fresh.
  observer_initialized_ = false;
  wrench_filter_initialized_ = false;
  qd_filter_initialized_ = false;
  tau_filter_initialized_ = false;
  p_integral_.setZero();
  r_.setZero();
  tau_ext_.setZero();
  wrench_filtered_.setZero();
  qd_filtered_.setZero();
  tau_filtered_.setZero();
  exported_wrench_.fill(0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WrenchEstimatorBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  exported_wrench_.fill(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type WrenchEstimatorBroadcaster::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

void WrenchEstimatorBroadcaster::compute_external_torque_from_full_torque(double dt)
{
  // ── Dynamics: M, C, g, and forward kinematics ───────────────────────
  pinocchio::computeAllTerms(model_, *data_, q_, qd_filtered_);
  // computeAllTerms gives us data_.M (upper triangular), data_.C and the
  // forward kinematics. Mirror M to be fully symmetric.
  data_->M.triangularView<Eigen::StrictlyLower>() =
    data_->M.transpose().triangularView<Eigen::StrictlyLower>();
  // Generalized gravity (data_.g) — computeAllTerms populates this too.

  // β = g(q) - C(q,qd)^T * qd
  beta_.noalias() = data_->g;
  beta_.noalias() -= data_->C.transpose() * qd_filtered_;

  // Generalized momentum
  p_.noalias() = data_->M * qd_filtered_;

  // ── Momentum observer (implicit Euler) ──────────────────────────────
  // dp/dt = tau + tau_ext - C*qd - g + (C+C^T)*qd = tau + tau_ext + C^T*qd - g
  // ⇒ tau_ext = dp/dt - γ,  γ := tau - β = tau + C^T*qd - g
  //
  // Estimator: dp̂/dt = γ + r̂,  r̂ = K_O ⊙ (p - p̂)
  //
  // Backward Euler (unconditionally stable for any K_O > 0):
  //   p̂_{k+1} = p̂_k + (γ_k + r̂_{k+1})·dt
  //   r̂_{k+1} = K_O ⊙ (p_{k+1} - p̂_{k+1})
  //   ⇒ r̂_{k+1} = K_O / (1 + K_O·dt) ⊙ (p_{k+1} - p̂_k - γ_k·dt)
  if (!observer_initialized_)
  {
    p_integral_ = p_;
    r_.setZero();
    observer_initialized_ = true;
  }
  else if (dt > 0.0)
  {
    const Eigen::VectorXd gamma_k = tau_filtered_ - beta_;
    const Eigen::VectorXd predicted = p_integral_ + gamma_k * dt;
    const Eigen::VectorXd residual = p_ - predicted;
    r_ = (K_O_.array() / (1.0 + K_O_.array() * dt) * residual.array()).matrix();
    p_integral_ = predicted + r_ * dt;
  }
  tau_ext_ = r_;
}

controller_interface::return_type WrenchEstimatorBroadcaster::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // ── Read joint state ────────────────────────────────────────────────
  // Missing values (empty optional) fall back to the previous reading; for
  // the very first cycle the buffers were zero-initialised in on_activate.
  for (std::size_t j = 0; j < n_joints_; ++j)
  {
    const auto idx = static_cast<Eigen::Index>(j);
    if (auto v = state_interfaces_[joint_si_idx_[j][0]].get_optional<double>())
    {
      q_(idx) = *v;
    }
    if (auto v = state_interfaces_[joint_si_idx_[j][1]].get_optional<double>())
    {
      qd_(idx) = *v;
    }
    if (auto v = state_interfaces_[joint_si_idx_[j][2]].get_optional<double>())
    {
      tau_meas_(idx) = torque_scale_[j] * *v;
    }
  }

  // ── Optional pre-filters on qd and tau ──────────────────────────────
  {
    const double a = lpf_alpha(dt, params_.joint_velocity_filter_cutoff_hz);
    if (a >= 1.0 || !qd_filter_initialized_)
    {
      qd_filtered_ = qd_;
      qd_filter_initialized_ = true;
    }
    else
    {
      qd_filtered_ = (1.0 - a) * qd_filtered_ + a * qd_;
    }
  }
  {
    const double a = lpf_alpha(dt, params_.joint_torque_filter_cutoff_hz);
    if (a >= 1.0 || !tau_filter_initialized_)
    {
      tau_filtered_ = tau_meas_;
      tau_filter_initialized_ = true;
    }
    else
    {
      tau_filtered_ = (1.0 - a) * tau_filtered_ + a * tau_meas_;
    }
  }

  // ── External joint torque ───────────────────────────────────────────
  // When the source already reports the external/disturbance torque, the whole
  // dynamics + observer block below is not just unnecessary but wrong: the
  // robot controller has already subtracted its own model, so subtracting
  // gravity again here would report roughly the negative of the load.
  if (params_.torque_is_external)
  {
    tau_ext_ = tau_filtered_;
  }
  else
  {
    compute_external_torque_from_full_torque(dt);
  }

  // ── Jacobian at tip in tip-local frame ──────────────────────────────
  pinocchio::computeJointJacobians(model_, *data_, q_);
  pinocchio::updateFramePlacement(model_, *data_, tip_frame_id_);
  J_.setZero();
  pinocchio::getFrameJacobian(model_, *data_, tip_frame_id_, pinocchio::LOCAL, J_);

  // ── Damped right-pseudo-inverse of J^T, gated on conditioning ───────
  // F = (JJ^T + αI)^-1 J τ_ext  when σ_min(J) ≥ threshold
  // Otherwise hold the last filtered output (set raw to the filter state so
  // the LPF + deadband downstream naturally freeze).
  JJt_.noalias() = J_ * J_.transpose();
  eig_solver_.compute(JJt_, Eigen::EigenvaluesOnly);
  const double lambda_min = std::max(0.0, eig_solver_.eigenvalues().minCoeff());
  const double sigma_min = std::sqrt(lambda_min);

  if (sigma_min < params_.singularity_threshold)
  {
    wrench_raw_ = wrench_filtered_;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Wrench gated: sigma_min(J)=%.4e < threshold=%.4e; holding last value.", sigma_min,
      params_.singularity_threshold);
  }
  else
  {
    JJt_alphaI_ = JJt_;
    JJt_alphaI_.diagonal().array() += params_.pinv_damping;
    wrench_raw_ = JJt_alphaI_.ldlt().solve(J_ * tau_ext_);
  }

  // ── Output LPF ──────────────────────────────────────────────────────
  {
    const double a = lpf_alpha(dt, params_.wrench_filter_cutoff_hz);
    if (a >= 1.0 || !wrench_filter_initialized_)
    {
      wrench_filtered_ = wrench_raw_;
      wrench_filter_initialized_ = true;
    }
    else
    {
      wrench_filtered_ = (1.0 - a) * wrench_filtered_ + a * wrench_raw_;
    }
  }

  // ── Per-axis deadband ───────────────────────────────────────────────
  Eigen::Matrix<double, kCartesianDim, 1> wrench_out = wrench_filtered_;
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(kCartesianDim); ++i)
  {
    if (std::abs(wrench_out(i)) < wrench_deadband_(i))
    {
      wrench_out(i) = 0.0;
    }
  }

  // ── Write exported state interfaces ─────────────────────────────────
  for (std::size_t i = 0; i < kCartesianDim; ++i)
  {
    exported_wrench_[i] = wrench_out(static_cast<Eigen::Index>(i));
  }

  // ── Publish for visibility ──────────────────────────────────────────
  if (rt_wrench_pub_ && rt_wrench_pub_->trylock())
  {
    auto & msg = rt_wrench_pub_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = params_.tip_link;
    msg.wrench.force.x = exported_wrench_[0];
    msg.wrench.force.y = exported_wrench_[1];
    msg.wrench.force.z = exported_wrench_[2];
    msg.wrench.torque.x = exported_wrench_[3];
    msg.wrench.torque.y = exported_wrench_[4];
    msg.wrench.torque.z = exported_wrench_[5];
    rt_wrench_pub_->unlockAndPublish();
  }

  // ── Publish the payload the wrench implies ──────────────────────────
  if (rt_payload_pub_ && rt_payload_pub_->trylock())
  {
    const Eigen::Vector3d f = wrench_out.head<3>();
    const Eigen::Vector3d t = wrench_out.tail<3>();
    const double f_norm = f.norm();

    auto & msg = rt_payload_pub_->msg_;
    msg.header.stamp = time;
    msg.header.frame_id = params_.tip_link;
    msg.inertia.m = f_norm / gravity_magnitude_;

    // From tau = r x F, the minimum-norm solution is r = (F x tau)/|F|^2.
    if (f_norm > 1e-6)
    {
      const Eigen::Vector3d com = f.cross(t) / (f_norm * f_norm);
      msg.inertia.com.x = com.x();
      msg.inertia.com.y = com.y();
      msg.inertia.com.z = com.z();
    }
    else
    {
      msg.inertia.com.x = 0.0;
      msg.inertia.com.y = 0.0;
      msg.inertia.com.z = 0.0;
    }

    msg.inertia.ixx = 0.0;
    msg.inertia.ixy = 0.0;
    msg.inertia.ixz = 0.0;
    msg.inertia.iyy = 0.0;
    msg.inertia.iyz = 0.0;
    msg.inertia.izz = 0.0;

    rt_payload_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_torque_wrench_estimator

PLUGINLIB_EXPORT_CLASS(
  joint_torque_wrench_estimator::WrenchEstimatorBroadcaster,
  controller_interface::ChainableControllerInterface)
