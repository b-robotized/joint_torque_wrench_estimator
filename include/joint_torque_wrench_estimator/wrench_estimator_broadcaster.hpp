// Copyright 2026 B-Robotized
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef JOINT_TORQUE_WRENCH_ESTIMATOR__WRENCH_ESTIMATOR_BROADCASTER_HPP_
#define JOINT_TORQUE_WRENCH_ESTIMATOR__WRENCH_ESTIMATOR_BROADCASTER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <pinocchio/multibody.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "joint_torque_wrench_estimator/wrench_estimator_broadcaster_parameters.hpp"

namespace joint_torque_wrench_estimator
{

class WrenchEstimatorBroadcaster : public controller_interface::ChainableControllerInterface
{
public:
  WrenchEstimatorBroadcaster();
  ~WrenchEstimatorBroadcaster() override = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

private:
  static constexpr std::size_t kCartesianDim = 6;

  void compute_external_torque_from_full_torque(double dt);

  // Parameters
  std::shared_ptr<wrench_estimator_broadcaster::ParamListener> param_listener_;
  wrench_estimator_broadcaster::Params params_;

  // Robot model (reduced to base_link -> tip_link chain in on_configure).
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  std::vector<std::string> joint_names_;
  std::size_t n_joints_{0};

  std::vector<std::string> torque_interface_names_;
  Eigen::VectorXd torque_scale_;
  pinocchio::FrameIndex base_frame_id_{0};
  pinocchio::FrameIndex tip_frame_id_{0};

  // Mapping from joint index in joint_names_ to indices into state_interfaces_.
  // joint_si_idx_[i] = {pos_idx, vel_idx, eff_idx}.
  std::vector<std::array<std::size_t, 3>> joint_si_idx_;

  // Working buffers (allocated in on_configure).
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd qd_filtered_;
  Eigen::VectorXd tau_meas_;
  Eigen::VectorXd tau_filtered_;
  Eigen::VectorXd beta_;
  Eigen::VectorXd p_;
  Eigen::VectorXd p_integral_;
  Eigen::VectorXd r_;
  Eigen::VectorXd tau_ext_;
  Eigen::VectorXd K_O_;
  Eigen::Matrix<double, kCartesianDim, Eigen::Dynamic> J_;
  Eigen::Matrix<double, kCartesianDim, kCartesianDim> JJt_;
  Eigen::Matrix<double, kCartesianDim, kCartesianDim> JJt_alphaI_;
  Eigen::Matrix<double, kCartesianDim, kCartesianDim> I6_;
  // Pre-allocated eigen solver for the singularity gate (no heap on hot path).
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, kCartesianDim, kCartesianDim>> eig_solver_;
  Eigen::Matrix<double, kCartesianDim, 1> wrench_raw_;
  Eigen::Matrix<double, kCartesianDim, 1> wrench_filtered_;
  Eigen::Matrix<double, kCartesianDim, 1> wrench_deadband_;

  // Exported FTS state-interface backing storage.
  // Order: Fx, Fy, Fz, Tx, Ty, Tz
  std::array<double, kCartesianDim> exported_wrench_{};

  // State flags
  bool observer_initialized_{false};
  bool wrench_filter_initialized_{false};
  bool qd_filter_initialized_{false};
  bool tau_filter_initialized_{false};

  // Publisher
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> wrench_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
    rt_wrench_pub_;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::InertiaStamped>> payload_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::InertiaStamped>>
    rt_payload_pub_;

  // Normalized magnitude of gravity, from gravity_vector. Converts |F| to a mass.
  double gravity_magnitude_{9.81};
};

}  // namespace joint_torque_wrench_estimator

#endif  // JOINT_TORQUE_WRENCH_ESTIMATOR__WRENCH_ESTIMATOR_BROADCASTER_HPP_
