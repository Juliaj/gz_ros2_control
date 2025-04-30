// Copyright 2025 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/joint_pid_helper.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <control_toolbox/pid.hpp>

using gz_ros2_control::JointPosVelPidHelper;

// Fix std::clamp issue
template<class T>
constexpr const T & custom_clamp(const T & v, const T & lo, const T & hi)
{
  return v < lo ? lo : hi < v ? hi : v;
}

// Test PID configuration
TEST(TestJointPosVelPidHelper, TestConfigurePid)
{
  // Create a PID controller
  control_toolbox::Pid pid;

  // Configure with test values
  JointPosVelPidHelper::configure_pid(
    pid,
    1.0,    // P gain
    0.5,    // I gain
    0.1,    // D gain
    2.0,    // I max
    -2.0,   // I min
    10.0,   // cmd max (not directly used in control_toolbox::Pid)
    -10.0,  // cmd min (not directly used in control_toolbox::Pid)
    0.2     // cmd offset (not directly used in control_toolbox::Pid)
  );

  // Verify values were set correctly
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);

  EXPECT_DOUBLE_EQ(p, 1.0);
  EXPECT_DOUBLE_EQ(i, 0.5);
  EXPECT_DOUBLE_EQ(d, 0.1);
  EXPECT_DOUBLE_EQ(i_max, 2.0);
  EXPECT_DOUBLE_EQ(i_min, -2.0);
  // Note: command max/min/offset aren't directly supported in control_toolbox::Pid
}

// Test velocity target force calculation
TEST(TestJointPosVelPidHelper, TestVelocityTargetForce)
{
  // Create and configure a PID controller
  control_toolbox::Pid pid;
  pid.initialize(1.0, 0.1, 0.0, 1.0, -1.0);

  // Set up test values
  double current_velocity = 2.0;
  double target_velocity = 5.0;
  double max_velocity = 10.0;

  // Create period (100ms)
  rclcpp::Duration period = rclcpp::Duration::from_seconds(0.1);

  // Calculate target force
  double target_force = JointPosVelPidHelper::calculate_velocity_target_force(
    pid, current_velocity, target_velocity, max_velocity, period);

  // For this simple test case with control_toolbox::Pid:
  // Error = target - current = 5.0 - 2.0 = 3.0
  // P-term = 1.0 * 3.0 = 3.0
  // Since we're using a simple P controller, expect around 3.0
  // (There will be some I-term buildup too)
  EXPECT_GT(target_force, 2.9);
  EXPECT_LT(target_force, 3.1);
}

// Test position target force calculation with cascade control
TEST(TestJointPosVelPidHelper, TestPositionTargetForceCascade)
{
  // Create and configure PID controllers
  control_toolbox::Pid pos_pid;
  control_toolbox::Pid vel_pid;

  // Position -> Velocity controller
  pos_pid.initialize(2.0, 0.0, 0.0, 0.0, 0.0);

  // Velocity -> Effort controller
  vel_pid.initialize(1.0, 0.0, 0.0, 0.0, 0.0);

  // Set up test values
  double current_position = 1.0;
  double target_position = 3.0;
  double current_velocity = 0.0;
  double lower_limit = -5.0;
  double upper_limit = 5.0;
  double max_velocity = 10.0;
  bool use_cascade = true;

  // Create period (100ms)
  rclcpp::Duration period = rclcpp::Duration::from_seconds(0.1);

  // Calculate target force
  double target_force = JointPosVelPidHelper::calculate_position_target_force(
    pos_pid, vel_pid, current_position, target_position, current_velocity,
    lower_limit, upper_limit, max_velocity, use_cascade, period);

  // For cascade mode with control_toolbox::Pid:
  // Pos error = target - current = 3.0 - 1.0 = 2.0
  // Target vel = pos_p_gain * pos_error = 2.0 * 2.0 = 4.0
  // Vel error = target - current = 4.0 - 0.0 = 4.0
  // Target force = vel_p_gain * vel_error = 1.0 * 4.0 = 4.0
  EXPECT_NEAR(target_force, 4.0, 0.01);
}

// Test position target force calculation without cascade control
TEST(TestJointPosVelPidHelper, TestPositionTargetForceDirect)
{
  // Create and configure PID controllers
  control_toolbox::Pid pos_pid;
  control_toolbox::Pid vel_pid;

  // Position controller (not used in direct mode)
  pos_pid.initialize(2.0, 0.0, 0.0, 0.0, 0.0);

  // Velocity -> Effort controller
  vel_pid.initialize(1.0, 0.0, 0.0, 0.0, 0.0);

  // Set up test values
  double current_position = 1.0;
  double target_position = 3.0;
  double current_velocity = 0.0;
  double lower_limit = -5.0;
  double upper_limit = 5.0;
  double max_velocity = 10.0;
  bool use_cascade = false;

  // Create period (100ms)
  rclcpp::Duration period = rclcpp::Duration::from_seconds(0.1);

  // Calculate target force
  double target_force = JointPosVelPidHelper::calculate_position_target_force(
    pos_pid, vel_pid, current_position, target_position, current_velocity,
    lower_limit, upper_limit, max_velocity, use_cascade, period);

  // For direct mode with control_toolbox::Pid:
  // Pos error = target - current = 3.0 - 1.0 = 2.0
  // Target force = vel_p_gain * pos_error = 1.0 * 2.0 = 2.0
  EXPECT_NEAR(target_force, 2.0, 0.01);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
