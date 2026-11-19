// Copyright 2026 B-Robotized
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <gmock/gmock.h>

#include <set>
#include <string>
#include <vector>

#include <controller_interface/controller_interface_params.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "joint_torque_wrench_estimator/wrench_estimator_broadcaster.hpp"

namespace
{
// Two-link arm with one fixed joint above the chain.
constexpr const char * kUrdf = R"(
<robot name="two_link">
  <link name="world"/>
  <link name="base_link">
    <inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <link name="link_1">
    <inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <link name="tool0">
    <inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <joint name="world_joint" type="fixed">
    <parent link="world"/><child link="base_link"/>
  </joint>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/><child link="link_1"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/><child link="tool0"/>
    <origin xyz="0.5 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" effort="100" velocity="1"/>
  </joint>
</robot>
)";
}  // namespace

class WrenchEstimatorBroadcasterTest : public ::testing::Test
{
public:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }
};

TEST_F(WrenchEstimatorBroadcasterTest, reconfigure_keeps_state_interfaces_unique)
{
  joint_torque_wrench_estimator::WrenchEstimatorBroadcaster controller;

  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "wrench_estimator";
  params.robot_description = kUrdf;
  params.update_rate = 100;
  params.controller_manager_update_rate = 100;
  params.node_options.parameter_overrides({{"base_link", "base_link"}, {"tip_link", "tool0"}});
  ASSERT_EQ(controller.init(params), controller_interface::return_type::OK);

  const auto expected =
    std::vector<std::string>{"joint_1/position", "joint_1/velocity", "joint_1/effort",
                             "joint_2/position", "joint_2/velocity", "joint_2/effort"};

  ASSERT_EQ(
    controller.get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller.state_interface_configuration().names, expected);

  ASSERT_EQ(
    controller.get_node()->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_EQ(
    controller.get_node()->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller.state_interface_configuration().names, expected);
}
